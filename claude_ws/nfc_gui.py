#!/usr/bin/env python3
"""PN532 小上位机 —— nfcpy 当后端，浏览器当界面。

用法:
    pip install nfcpy
    python3 nfc_gui.py [设备] [端口]
    python3 nfc_gui.py                     # 自动找串口，认不出就在页面上选
    python3 nfc_gui.py /dev/cu.usbmodem0000696118491       # macOS
    python3 nfc_gui.py /dev/ttyUSB0                        # Ubuntu
    python3 nfc_gui.py COM3                                # Windows
    python3 nfc_gui.py --selftest          # 无硬件自检

设备省略时扫一遍串口，恰好只有一个就直接用；页面顶部也能随时换。只监听 127.0.0.1。
"""
import contextlib
import glob
import json
import os
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

try:
    import ndef  # ndeflib 的导入名就叫 ndef
    import nfc
except ImportError as e:
    sys.exit("缺依赖：%s —— pip install nfcpy pyserial libusb1 ndeflib" % e)

if sys.platform == "darwin":
    # macOS 专属：nfcpy 只认 ttyUSB/ACM/AMA 和 cu.usbserial，usbmodem、SLAB、
    # wchusbserial 这些名字既过不了它的设备串语法（字符集不含点、下划线），也不在
    # 兜底正则里，于是 open() 找不到设备并抛 ENODEV。Linux 上没这毛病，别去动它。
    import nfc.clf.transport
    nfc.clf.transport.PATH = re.compile(
        r'^([a-z]+)(?::|)([a-zA-Z0-9._\-]+|)(?::|)([a-zA-Z0-9]+|)$')


@contextlib.contextmanager
def no_baud_escalation():
    """掐掉 nfcpy 在 Linux 上的 921600 换挡（pn532.py:424-431）。

    那几行是 `stty -F /dev/ttyACM0 921600`，等于另起一个 sh 进程去 open/close 我们
    正开着的口，还顺手改掉共享的 termios。CDC 桥接芯片（SEGGER J-Link 这种）扛不住
    就直接从 USB 总线掉下去、设备重枚举，页面上的串口当场消失。

    stty 一律"失败"的话，那个 for 循环不会 break，走完 baudrate 正好停在最后一个
    115200，`if baudrate > 115200` 不成立，就不会给 PN532 发换挡命令 —— 和 macOS
    的行为一致（macOS 因为 sys.platform 判断从来不升速，一直跑 115200 没出过事）。
    """
    if not sys.platform.startswith("linux"):
        yield
        return
    # ponytail: 全局改 os.system，但只在 open 那一瞬间、且本进程别处不调 os.system；
    # 要更干净就只能去改 nfcpy 源码了。
    real = os.system
    os.system = lambda cmd: 1 if "stty" in cmd else real(cmd)
    try:
        yield
    finally:
        os.system = real

LOCK = threading.Lock()
WAKE = threading.Event()  # 换串口时叫醒 worker，免得白等满一轮
STATE = {"reader": "启动中…", "device": None, "tag": None, "dump": None,
         "log": [], "pending": None}
CMDS = ("write", "format", "dump", "open")


def log(msg):
    with LOCK:
        STATE["log"].insert(0, time.strftime("%H:%M:%S ") + msg)
        del STATE["log"][30:]


def snapshot():
    with LOCK:
        return json.dumps({k: STATE[k] for k in STATE if k != "pending"})


def record_text(r):
    """ndeflib 的 str() 很啰嗦，优先取记录本身的内容。"""
    for attr in ("text", "uri"):
        v = getattr(r, attr, None)
        if isinstance(v, str):
            return v
    return str(r)


def tag_info(tag):
    info = {
        "type": tag.type,
        "product": tag.product or "未知（可能是兼容片，不认 GET_VERSION）",
        "uid": tag.identifier.hex(" ").upper(),
        "ndef": None,
    }
    n = tag.ndef
    if n is None:
        return info
    info["ndef"] = {
        "capacity": n.capacity,
        "length": n.length,
        "readable": n.is_readable,
        "writable": n.is_writeable,
    }
    try:
        info["records"] = [record_text(r) for r in n.records]
    except Exception as e:
        info["records"] = ["读取失败: %s" % e]
    return info


def on_tag(tag):
    """每感应到一次卡就走这里：先刷新状态，再执行排队中的命令。

    必须 return True：nfcpy 约定的方向是反的——on-connect 返回 False 表示
    connect() 立刻返回，只有返回 True 才会守到卡离开。返回 False 会让读卡器
    陷入 close/reopen 空转，页面上的「读卡器」就会反复闪。
    """
    with LOCK:
        pending, STATE["pending"] = STATE["pending"], None
    try:
        info = tag_info(tag)  # 读卡可能慢，别占着锁
        with LOCK:
            STATE["tag"] = info
        if pending:
            if pending["cmd"] == "dump":
                with LOCK:
                    STATE["dump"] = tag.dump()
                log("页转储完成")
            elif pending["cmd"] == "format":
                tag.format()
                log("格式化完成，NDEF 区已建立")
            elif pending["cmd"] == "write":
                if tag.ndef is None:
                    raise ValueError("这张卡没有 NDEF 区，先点「格式化」")
                tag.ndef.records = [ndef.TextRecord(pending.get("text") or "")]
                log("写入完成")
    except Exception as e:
        log("操作失败：%s" % e)
    return True


def list_ports():
    """候选串口，不锁死名字：Windows/Linux 交给 pyserial 枚举，macOS 扫 /dev。

    macOS 上不能用 pyserial 的 comports()：它靠 ctypes 调 IOKit/CoreFoundation，
    程序被 PyInstaller 冻结后那些框架路径会被改坏，列出来永远是空。Linux 的
    list_ports_linux 是纯读 /sys，没这问题，而且 ttyS/ttyUSB/ttyXRUSB/ttyACM/
    ttyAMA/rfcomm/ttyAP 全认，还顺手滤掉不存在的内置口，比手写通配靠谱。
    """
    if sys.platform == "darwin":
        # 不要写 /dev/cu.usb*：CP2102 叫 cu.SLAB_USBtoUART、CH340 叫
        # cu.wchusbserial*，一律漏掉。cu 而不是 tty，是 cu 不等 DCD、插上就能开。
        return sorted(p for p in glob.glob("/dev/cu.*")
                      if "Bluetooth" not in p and "debug" not in p)
    from serial.tools import list_ports as lp
    return sorted(p.device for p in lp.comports()) or sorted(
        {p for pat in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*")
         for p in glob.glob(pat)})


def device_string(arg):
    """把 /dev/cu.usbmodemXXXX 或 COM3 这样的路径翻成 nfcpy 的设备串。"""
    if not arg or ":" in arg:  # 已经是 nfcpy 原生设备串，原样透传
        return arg
    base = os.path.basename(arg)
    if re.match(r'^COM\d+$', base, re.I):
        return "com:%s:pn532" % base
    return "tty:%s:pn532" % base


def open_hint(e):
    """把「打不开串口」翻译成人话。nfcpy 只抛裸 errno，新手看不出要干嘛。"""
    s = str(e)
    if isinstance(e, PermissionError) or "Permission denied" in s:
        return "（没串口权限：Ubuntu 上 sudo usermod -aG dialout $USER 后重新登录）"
    if isinstance(e, FileNotFoundError) or "No such file" in s:
        return "（设备名不存在：先 ls /dev/ttyUSB* /dev/ttyACM* /dev/cu.* 看真名）"
    return ""


def worker():
    """守着串口。开哪个由 STATE["device"] 决定，页面可以随时换。"""
    clf, current = None, None
    while True:
        with LOCK:
            want = STATE["device"]
        if want != current:
            if clf is not None:
                try:
                    clf.close()
                except Exception:
                    pass
            clf, current = None, None
            if want:
                try:
                    with no_baud_escalation():
                        clf = nfc.ContactlessFrontend(device_string(want))
                    current = want
                    with LOCK:
                        STATE["reader"] = "就绪：%s" % clf
                except Exception as e:
                    with LOCK:
                        STATE["reader"] = "打开失败：%s%s" % (e, open_hint(e))
            else:
                with LOCK:
                    STATE["reader"] = "未选择串口，请在页面上选一个"
        if clf is None:
            WAKE.wait(2)
            WAKE.clear()
            continue
        try:
            clf.connect(rdwr={"on-connect": on_tag})
        except Exception as e:
            with LOCK:
                STATE["reader"] = "断开了：%s" % e
            try:
                clf.close()
            except Exception:
                pass
            clf, current = None, None
            WAKE.wait(1)
            WAKE.clear()


PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>PN532 小上位机</title><style>
body{font:14px/1.6 -apple-system,system-ui,sans-serif;background:#14161a;color:#dfe3e8;margin:0;padding:24px}
.wrap{max-width:760px;margin:0 auto}h1{font-size:18px;margin:0 0 4px}
.sub{color:#8b949e;font-size:12px;margin-bottom:20px}
.card{background:#1c1f26;border:1px solid #2a2f38;border-radius:10px;padding:16px;margin-bottom:14px}
.row{display:flex;justify-content:space-between;gap:16px;padding:4px 0;border-bottom:1px dashed #2a2f38}
.row:last-child{border:0}.k{color:#8b949e;flex:0 0 auto}
textarea{width:100%;box-sizing:border-box;background:#0f1115;color:#dfe3e8;border:1px solid #2a2f38;border-radius:6px;padding:8px;font-family:ui-monospace,monospace}
button{background:#2f81f7;color:#fff;border:0;border-radius:6px;padding:8px 14px;cursor:pointer;margin:10px 8px 0 0}
button.ghost{background:#2a2f38}pre{background:#0f1115;border-radius:6px;padding:10px;overflow:auto;max-height:260px;font-size:12px;white-space:pre-wrap}
.log{color:#8b949e;font-size:12px}
select{background:#0f1115;color:#dfe3e8;border:1px solid #2a2f38;border-radius:6px;padding:6px;max-width:300px}
.row button{margin:0 0 0 8px;padding:6px 10px}
</style></head><body><div class="wrap">
<h1>PN532 小上位机</h1><div class="sub">nfcpy 后端 · 只监听 127.0.0.1 · 点完按钮再把卡贴上去（已经贴着的先拿开再放一次）</div>
<div class="card">
<div class="row"><span class="k">串口设备</span><span><select id="port"></select>
<button class="ghost" onclick="loadPorts()">刷新</button>
<button onclick="pickPort()">连接</button></span></div>
<div class="row"><span class="k">读卡器</span><span id="reader">…</span></div>
<div class="row"><span class="k">卡型号</span><span id="product">—</span></div>
<div class="row"><span class="k">类型</span><span id="type">—</span></div>
<div class="row"><span class="k">UID</span><span id="uid">—</span></div>
<div class="row"><span class="k">NDEF 容量</span><span id="cap">—</span></div>
<div class="row"><span class="k">已用 / 可写</span><span id="used">—</span></div>
<div class="row"><span class="k">当前内容</span><span id="recs">—</span></div></div>
<div class="card"><div class="k">写入 NDEF 文本</div>
<textarea id="text" rows="3" placeholder="要写进卡里的文字"></textarea>
<button onclick="send('write')">写入</button>
<button class="ghost" onclick="send('format')">格式化</button>
<button class="ghost" onclick="send('dump')">读页转储</button>
<pre id="dump" style="display:none"></pre></div>
<div class="card"><div class="k">日志</div><div class="log" id="log">—</div></div>
</div><script>
async function send(cmd,device){
  const body = {cmd:cmd,text:document.getElementById('text').value};
  if(device) body.device = device;
  const r = await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)});
  if(!r.ok) alert('命令被拒绝');
}
async function loadPorts(){
  const sel = document.getElementById('port');
  const want = sel.dataset.want || '';
  const r = await (await fetch('/api/ports')).json();
  sel.innerHTML = r.ports.map(function(p){return '<option>'+p+'</option>'}).join('')
    || '<option value="">（没找到串口）</option>';
  if([].some.call(sel.options,function(o){return o.value===want})) sel.value = want;
}
function pickPort(){
  const v = document.getElementById('port').value;
  if(v) send('open',v);
}
function set(id,v){document.getElementById(id).textContent = (v===null||v===undefined)?'—':v}
async function tick(){
  const s = await (await fetch('/api/state')).json();
  set('reader',s.reader);
  const t = s.tag;
  set('product',t?t.product:null); set('type',t?t.type:null); set('uid',t?t.uid:null);
  if(t && t.ndef){
    set('cap',t.ndef.capacity+' 字节');
    set('used',t.ndef.length+' / '+(t.ndef.writable?'可写':'只读'));
    set('recs',(t.records||[]).join(' | ')||'（空）');
  } else if(t){
    set('cap','无 NDEF 区'); set('used',null); set('recs','先点「格式化」');
  } else { set('cap',null); set('used',null); set('recs',null); }
  const d = document.getElementById('dump');
  if(s.dump && s.dump.length){ d.style.display='block'; d.textContent=s.dump.join('\\n'); }
  if(s.device) document.getElementById('port').dataset.want = s.device;
  document.getElementById('log').innerHTML = s.log.map(function(x){return '<div>'+x+'</div>'}).join('') || '—';
}
setInterval(tick,700); tick(); loadPorts();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif path == "/api/state":
            self._send(200, snapshot())
        elif path == "/api/ports":
            self._send(200, json.dumps({"ports": list_ports()}))
        else:
            self._send(404, "{}")

    def do_POST(self):
        if urlparse(self.path).path != "/api/cmd":
            return self._send(404, "{}")
        raw = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        req = json.loads(raw or b"{}")
        if req.get("cmd") not in CMDS:
            return self._send(400, '{"error":"未知命令"}')
        if req["cmd"] == "open":
            with LOCK:
                STATE["device"] = req.get("device") or None
            WAKE.set()
            log("切换串口：%s" % (req.get("device") or "（无）"))
            return self._send(200, '{"ok":true}')
        with LOCK:
            STATE["pending"] = req
        log("已排队：%s（把卡贴上去）" % req["cmd"])
        self._send(200, '{"ok":true}')

    def log_message(self, *a):
        pass


def selftest():
    class FakeNDEF:
        capacity, length, is_readable, is_writeable = 144, 0, True, True
        records = []

    class FakeTag:
        type = "Type2Tag"
        product = "NXP NTAG213"
        identifier = bytes.fromhex("1DF290C10B1080")
        ndef = FakeNDEF()

        def dump(self):
            return ["page 0: 1D F2 90 C1", "page 1: 0B 10 80 48"]

    # 假对象必须和真 NDEF 的属性名对齐——上一版就在这里编了个不存在的 readable
    fake = {a for a in vars(FakeNDEF) if not a.startswith("_")}
    real = set(dir(nfc.tag.Tag.NDEF))
    assert fake <= real, "假对象编了真 NDEF 没有的属性: %s" % (fake - real)

    assert device_string("/dev/cu.usbmodem0000696118491") == "tty:cu.usbmodem0000696118491:pn532"
    assert device_string("/dev/ttyUSB0") == "tty:ttyUSB0:pn532"
    assert device_string("COM3") == "com:COM3:pn532"
    assert device_string("tty:USB0:pn532") == "tty:USB0:pn532", "原生设备串要原样透传"
    assert device_string(None) is None

    real_system = os.system
    with no_baud_escalation():
        if sys.platform.startswith("linux"):
            assert os.system("stty -F /dev/null 115200") == 1, "Linux 上 stty 必须被掐掉"
    assert os.system is real_system, "退出上下文后 os.system 没还原"

    assert "dialout" in open_hint(PermissionError(13, "Permission denied"))
    assert "ls /dev" in open_hint(FileNotFoundError(2, "No such file or directory"))
    assert open_hint(ValueError("别的错")) == "", "不认识的错别乱加提示"

    tag = FakeTag()
    info = tag_info(tag)
    assert info["product"] == "NXP NTAG213", info
    assert info["uid"] == "1D F2 90 C1 0B 10 80", info
    assert info["ndef"]["capacity"] == 144, info

    with LOCK:
        STATE["pending"] = {"cmd": "write", "text": "你好"}
    assert on_tag(tag) is True, "on-connect 必须返回 True，否则 connect() 会立刻返回"
    assert len(tag.ndef.records) == 1, tag.ndef.records
    assert tag.ndef.records[0].text == "你好", tag.ndef.records
    with LOCK:
        assert STATE["pending"] is None, "命令没被消费掉"

    with LOCK:
        STATE["pending"] = {"cmd": "dump"}
    on_tag(tag)
    assert STATE["dump"][1].startswith("page 1"), STATE["dump"]


def main():
    if "--selftest" in sys.argv:
        selftest()
        print("selftest OK")
        return
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    port = int(args[1]) if len(args) > 1 else 8765
    if args:
        device = args[0]
    else:
        found = list_ports()
        device = found[0] if len(found) == 1 else None  # 只有一个就自己接上
    with LOCK:
        STATE["device"] = device
    threading.Thread(target=worker, daemon=True).start()
    print("打开 http://127.0.0.1:%d" % port)
    if not device:
        print("没自动认出一个串口，请在页面上选一个")
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
