#!/bin/bash
# Jetson 侧: 配置 USB Gadget (functionfs)
# 在 Jetson 上以 root 运行一次即可

set -e

GADGET_DIR=/sys/kernel/config/usb_gadget/g_stress
FFS_MOUNT=/mnt/ffs_stress
SERIAL="1423823037322"

# 清理旧配置
if [ -d "$GADGET_DIR" ]; then
    echo "$GADGET_DIR" > "$GADGET_DIR/UDC" 2>/dev/null || true
    sleep 1
    [ -L "$GADGET_DIR/configs/c.1/ffs.stress0" ] && rm "$GADGET_DIR/configs/c.1/ffs.stress0"
    rmdir "$GADGET_DIR/configs/c.1/strings/0x409" 2>/dev/null || true
    rmdir "$GADGET_DIR/configs/c.1" 2>/dev/null || true
    rmdir "$GADGET_DIR/functions/ffs.stress0" 2>/dev/null || true
    rmdir "$GADGET_DIR/strings/0x409" 2>/dev/null || true
    rmdir "$GADGET_DIR" 2>/dev/null || true
fi

modprobe libcomposite 2>/dev/null || true

# 创建 gadget
mkdir "$GADGET_DIR"
echo "0x1d6b" > "$GADGET_DIR/idVendor"
echo "0x0104" > "$GADGET_DIR/idProduct"
echo "0x0200" > "$GADGET_DIR/bcdUSB"
echo "0xEF"   > "$GADGET_DIR/bDeviceClass"
echo "0x02"   > "$GADGET_DIR/bDeviceSubClass"
echo "0x01"   > "$GADGET_DIR/bDeviceProtocol"

# 字符串
mkdir "$GADGET_DIR/strings/0x409"
echo "DirectDriveTech" > "$GADGET_DIR/strings/0x409/manufacturer"
echo "TITA_Stress" > "$GADGET_DIR/strings/0x409/product"
echo "$SERIAL" > "$GADGET_DIR/strings/0x409/serialnumber"

# 配置
mkdir "$GADGET_DIR/configs/c.1"
mkdir "$GADGET_DIR/configs/c.1/strings/0x409"
echo "Bulk Stress Test" > "$GADGET_DIR/configs/c.1/strings/0x409/configuration"
echo "120" > "$GADGET_DIR/configs/c.1/MaxPower"

# functionfs function
mkdir "$GADGET_DIR/functions/ffs.stress0"
ln -s "$GADGET_DIR/functions/ffs.stress0" "$GADGET_DIR/configs/c.1/ffs.stress0"

# 挂载 functionfs
mkdir -p "$FFS_MOUNT"
mount -t functionfs stress0 "$FFS_MOUNT" 2>/dev/null || true

echo "Gadget 已配置, functionfs 已挂载到 $FFS_MOUNT"
echo "现在运行: sudo ./device_fs $FFS_MOUNT"
echo "等 device_fs 就绪后, 再绑定 UDC:"
echo "  echo $(ls /sys/class/udc | head -1) > $GADGET_DIR/UDC"
