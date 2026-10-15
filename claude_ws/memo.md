1. 热路径上跑emit
2. 所以你的 HID(kCritical) 和 NCM(kNormal) 两条流其实挤在同一条 TCP 连接上
  另外三个坑，都在这份脚本里

  1. 版本号写死 1.0.0（:78）

  喵~ dpkg -i 同版本照样重装，不会拦你，但你也永远分不清装的是哪次编译~nya

  2. 没有 DEBIAN/conffiles，/etc 下的配置每次都被硬盖

  喵~ :48 把 build/config、build/launch 整个塞进 /etc/adapter_unit/，而 y2_usb_identity_param.yaml 不在
  conffiles 里 → dpkg 当普通文件无条件覆盖~nya

  喵~ 这条对主人特别要命：你手改成 role: device 之后，只要再 dpkg -i 一次新包就被打回 host，然后 device
  板子悄悄变回 host，gadget 全没了~nya

  3. dpkg -i 依赖失败时 postinst 根本不跑

  喵~ 这时候包停在 unpacked，文件可能铺了一半、服务没重启，dpkg -l 看着像"装过了"~nya

  dpkg -s adapter-unit | grep Status     # 要看到 install ok installed

  喵~ 装的时候用 apt install ./adapter-unit.deb 取代 dpkg -i，依赖和配置阶段都会走完；已经卡住了就 apt-get
  -f install 收尾~nya


1. 	this->hid_fd_ = open("/dev/hidraw0", O_RDWR);
要从外部获取
