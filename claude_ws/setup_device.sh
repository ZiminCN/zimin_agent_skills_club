#!/bin/bash
# 从机 (USB device 端) — 先跑这个

# 1. 清理所有残留
sudo ip link set dev usb0 nomaster 2>/dev/null
sudo ip link del dbgbr0 2>/dev/null

# 清理所有口上残留的 192.168.42.x IP
for iface in $(ip -o link show | awk -F': ' '{print $2}'); do
    sudo ip addr del 192.168.42.1/24 dev "$iface" 2>/dev/null
    sudo ip addr del 192.168.42.2/24 dev "$iface" 2>/dev/null
done
sudo ip addr flush dev usb0 2>/dev/null

# 2. 重新枚举 USB gadget
sudo sh -c 'echo "" > /sys/kernel/config/usb_gadget/dbg/UDC' 2>/dev/null
sleep 1
sudo sh -c "echo $(ls /sys/class/udc/ | head -1) > /sys/kernel/config/usb_gadget/dbg/UDC"
sleep 2

# 3. 建桥
sudo ip link add name dbgbr0 type bridge
sudo ip link set dev dbgbr0 address c6:f9:a9:66:0a:7f
sudo ip addr add 192.168.42.2/24 broadcast + dev dbgbr0
sudo ip link set dbgbr0 up

# 4. 挂 usb0
sudo ip link set usb0 up
sudo ip link set dev usb0 master dbgbr0

# 5. 等链路稳定，干掉可能冒出来的 192.168.42.1（二次保险）
sleep 1
sudo ip addr del 192.168.42.1/24 dev dbgbr0 2>/dev/null

echo ""
echo "[device] done"
echo "[device] verify: ip addr show dbgbr0 | grep inet"
echo "[device] should show ONLY 192.168.42.2"
