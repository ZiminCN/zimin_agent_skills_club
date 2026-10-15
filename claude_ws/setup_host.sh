#!/bin/bash
# 主机 (USB host 端) — 从机跑完再跑这个

# 1. 主机不准当 USB device
sudo sh -c 'echo "" > /sys/kernel/config/usb_gadget/dbg/UDC' 2>/dev/null

# 2. 清理
sudo ip link set dev usb0 nomaster 2>/dev/null
sudo ip link set dev usb1 nomaster 2>/dev/null
sudo ip link del dbgbr0 2>/dev/null
sudo ip addr flush dev usb0 2>/dev/null
sudo ip addr flush dev usb1 2>/dev/null

# 3. 等 NCM 接口出现（最多等 5 秒）
NCM=""
for i in $(seq 1 10); do
    for iface in usb0 usb1; do
        if ip link show "$iface" 2>/dev/null | grep -q LOWER_UP; then
            NCM="$iface"
            break 2
        fi
    done
    sleep 1
done

# 退而求其次：找 cdc_ncm 驱动的口
if [ -z "$NCM" ]; then
    for iface in usb0 usb1; do
        DRV=$(readlink -f /sys/class/net/$iface/device/driver 2>/dev/null | xargs basename 2>/dev/null)
        if [ "$DRV" = "cdc_ncm" ]; then
            NCM="$iface"
            break
        fi
    done
fi

if [ -z "$NCM" ]; then
    echo "[host] ERROR: no NCM interface. avail usb:"
    ip link show 2>/dev/null | grep -E "^[0-9]+: usb"
    exit 1
fi

echo "[host] using interface: $NCM"

# 4. 建桥
sudo ip link add name dbgbr0 type bridge
sudo ip addr add 192.168.42.1/24 broadcast + dev dbgbr0
sudo ip link set dbgbr0 up

sudo ip link set "$NCM" up
sudo ip link set dev "$NCM" master dbgbr0

echo ""
echo "[host] done. try: ping 192.168.42.2"
