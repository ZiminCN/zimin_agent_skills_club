/*
 * g_bulk.c — Jetson USB Gadget: 裸 Bulk 双向端点
 *
 * 编译（在 Jetson 上）:
 *   make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
 *
 * 使用:
 *   sudo rmmod g_bulk_stress 2>/dev/null
 *   sudo insmod g_bulk.ko
 *   然后 RK3588 上: sudo ./host_libusb
 *
 * 卸载:
 *   sudo rmmod g_bulk_stress
 *
 * VID=0x1d6b, PID=0x0104
 * EP 0x01: Bulk OUT (Host→Device)
 * EP 0x82: Bulk IN  (Device→Host)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#define DRIVER_NAME     "g_bulk_stress"
#define DRIVER_DESC     "USB Bulk Stress Test"
#define BUF_SIZE        16384

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("BulkTest");

/* 设备描述符 */
static struct usb_device_descriptor device_desc = {
    .bLength            = USB_DT_DEVICE_SIZE,
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = cpu_to_le16(0x0200),
    .bDeviceClass       = 0xFF,
    .bDeviceSubClass    = 0x42,
    .bDeviceProtocol    = 0x01,
    .bMaxPacketSize0    = 64,
    .idVendor           = cpu_to_le16(0x1d6b),
    .idProduct          = cpu_to_le16(0x0104),
    .bcdDevice          = cpu_to_le16(0x0100),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
};

/* 字符串 */
static struct usb_string strings[] = {
    { .id = 1, .s = "BulkTest" },
    { .id = 2, .s = "Bulk Stress Gadget" },
    { },
};

static struct usb_gadget_strings stringtab = {
    .language = 0x0409,
    .strings  = strings,
};

static struct usb_gadget_strings *dev_strings[] = {
    &stringtab,
    NULL,
};

/* FS 描述符 */
static struct usb_interface_descriptor intf_desc_fs = {
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = 0xFF,
    .bInterfaceSubClass = 0x42,
    .bInterfaceProtocol = 0x01,
    .iInterface         = 0,
};

static struct usb_endpoint_descriptor ep_out_desc_fs = {
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = 0x01,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(64),
};

static struct usb_endpoint_descriptor ep_in_desc_fs = {
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = 0x82,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(64),
};

/* HS 描述符 */
static struct usb_interface_descriptor intf_desc_hs = {
    .bLength            = USB_DT_INTERFACE_SIZE,
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = 0xFF,
    .bInterfaceSubClass = 0x42,
    .bInterfaceProtocol = 0x01,
    .iInterface         = 0,
};

static struct usb_endpoint_descriptor ep_out_desc_hs = {
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = 0x01,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(512),
};

static struct usb_endpoint_descriptor ep_in_desc_hs = {
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = 0x82,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(512),
};

static struct usb_descriptor_header *fs_desc[] = {
    (struct usb_descriptor_header *)&intf_desc_fs,
    (struct usb_descriptor_header *)&ep_out_desc_fs,
    (struct usb_descriptor_header *)&ep_in_desc_fs,
    NULL,
};

static struct usb_descriptor_header *hs_desc[] = {
    (struct usb_descriptor_header *)&intf_desc_hs,
    (struct usb_descriptor_header *)&ep_out_desc_hs,
    (struct usb_descriptor_header *)&ep_in_desc_hs,
    NULL,
};

/*-------------------------------------------------------------------------*/

#define NUM_REQ 4

struct f_bulk {
    struct usb_function func;
    struct usb_ep *ep_in;
    struct usb_ep *ep_out;
    struct usb_request **in_req;
    struct usb_request **out_req;
    atomic64_t tx_bytes;
    atomic64_t rx_bytes;
};

static struct f_bulk *g_f_bulk;

static void bulk_in_complete(struct usb_ep *ep, struct usb_request *req)
{
    struct f_bulk *f = req->context;
    if (!f || req->status) return;

    atomic64_add(req->actual, &f->tx_bytes);
    memset(req->buf, 0xAA, BUF_SIZE);
    req->length = BUF_SIZE;
    if (usb_ep_queue(ep, req, GFP_ATOMIC))
        req->status = -ECONNRESET;
}

static void bulk_out_complete(struct usb_ep *ep, struct usb_request *req)
{
    struct f_bulk *f = req->context;
    if (!f || req->status) return;

    atomic64_add(req->actual, &f->rx_bytes);
    req->length = BUF_SIZE;
    if (usb_ep_queue(ep, req, GFP_ATOMIC))
        req->status = -ECONNRESET;
}

static int alloc_requests(struct f_bulk *f)
{
    int i, ret;

    f->in_req = kcalloc(NUM_REQ, sizeof(*f->in_req), GFP_KERNEL);
    f->out_req = kcalloc(NUM_REQ, sizeof(*f->out_req), GFP_KERNEL);
    if (!f->in_req || !f->out_req) return -ENOMEM;

    for (i = 0; i < NUM_REQ; i++) {
        /* IN */
        f->in_req[i] = usb_ep_alloc_request(f->ep_in, GFP_KERNEL);
        if (!f->in_req[i]) return -ENOMEM;
        f->in_req[i]->buf = kmalloc(BUF_SIZE, GFP_KERNEL);
        if (!f->in_req[i]->buf) return -ENOMEM;
        f->in_req[i]->length = BUF_SIZE;
        f->in_req[i]->context = f;
        f->in_req[i]->complete = bulk_in_complete;
        memset(f->in_req[i]->buf, 0xAA, BUF_SIZE);
        ret = usb_ep_queue(f->ep_in, f->in_req[i], GFP_KERNEL);
        if (ret) return ret;

        /* OUT */
        f->out_req[i] = usb_ep_alloc_request(f->ep_out, GFP_KERNEL);
        if (!f->out_req[i]) return -ENOMEM;
        f->out_req[i]->buf = kmalloc(BUF_SIZE, GFP_KERNEL);
        if (!f->out_req[i]->buf) return -ENOMEM;
        f->out_req[i]->length = BUF_SIZE;
        f->out_req[i]->context = f;
        f->out_req[i]->complete = bulk_out_complete;
        ret = usb_ep_queue(f->ep_out, f->out_req[i], GFP_KERNEL);
        if (ret) return ret;
    }
    return 0;
}

static void free_requests(struct f_bulk *f)
{
    int i;
    if (f->in_req) {
        for (i = 0; i < NUM_REQ; i++) {
            if (f->in_req[i]) {
                usb_ep_dequeue(f->ep_in, f->in_req[i]);
                kfree(f->in_req[i]->buf);
                usb_ep_free_request(f->ep_in, f->in_req[i]);
            }
        }
        kfree(f->in_req);
        f->in_req = NULL;
    }
    if (f->out_req) {
        for (i = 0; i < NUM_REQ; i++) {
            if (f->out_req[i]) {
                usb_ep_dequeue(f->ep_out, f->out_req[i]);
                kfree(f->out_req[i]->buf);
                usb_ep_free_request(f->ep_out, f->out_req[i]);
            }
        }
        kfree(f->out_req);
        f->out_req = NULL;
    }
}

static int bulk_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    struct f_bulk *fb = container_of(f, struct f_bulk, func);
    struct usb_composite_dev *cdev = f->config->cdev;
    int ret;

    fb->ep_in = usb_ep_autoconfig(cdev->gadget, &ep_in_desc_hs);
    fb->ep_out = usb_ep_autoconfig(cdev->gadget, &ep_out_desc_hs);
    if (!fb->ep_in || !fb->ep_out) {
        dev_err(&cdev->gadget->dev, "can't find endpoints\n");
        return -ENODEV;
    }

    ret = usb_ep_enable(fb->ep_in, &ep_in_desc_hs);
    if (ret) return ret;
    ret = usb_ep_enable(fb->ep_out, &ep_out_desc_hs);
    if (ret) { usb_ep_disable(fb->ep_in); return ret; }

    dev_info(&cdev->gadget->dev, "Bulk ep_in=0x%02x ep_out=0x%02x\n",
             fb->ep_in->address, fb->ep_out->address);
    return alloc_requests(fb);
}

static void bulk_disable(struct usb_function *f)
{
    struct f_bulk *fb = container_of(f, struct f_bulk, func);
    free_requests(fb);
    if (fb->ep_in)  usb_ep_disable(fb->ep_in);
    if (fb->ep_out) usb_ep_disable(fb->ep_out);
}

static int bulk_bind(struct usb_configuration *c, struct usb_function *f)
{
    return usb_assign_descriptors(f, fs_desc, hs_desc, NULL, NULL);
}

static struct usb_configuration config = {
    .label          = "Bulk Stress Config",
    .bConfigurationValue = 1,
    .bmAttributes   = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
    .MaxPower       = 120,
};

static struct f_bulk *bulk_alloc(void)
{
    struct f_bulk *fb = kzalloc(sizeof(*fb), GFP_KERNEL);
    if (!fb) return NULL;
    fb->func.name    = "bulk_stress";
    fb->func.bind    = bulk_bind;
    fb->func.set_alt = bulk_set_alt;
    fb->func.disable = bulk_disable;
    fb->func.strings = dev_strings;
    atomic64_set(&fb->tx_bytes, 0);
    atomic64_set(&fb->rx_bytes, 0);
    return fb;
}

static int composite_bind(struct usb_composite_dev *cdev)
{
    int ret = usb_string_ids_tab(cdev, strings);
    if (ret < 0) return ret;
    device_desc.iManufacturer = strings[0].id;
    device_desc.iProduct      = strings[1].id;
    ret = usb_add_config(cdev, &config, NULL);
    if (ret < 0) return ret;
    g_f_bulk = bulk_alloc();
    if (!g_f_bulk) return -ENOMEM;
    ret = usb_add_function(&config, &g_f_bulk->func);
    if (ret < 0) { kfree(g_f_bulk); g_f_bulk = NULL; }
    return ret;
}

static int composite_unbind(struct usb_composite_dev *cdev)
{
    kfree(g_f_bulk);
    g_f_bulk = NULL;
    return 0;
}

static struct usb_composite_driver bulk_driver = {
    .name      = DRIVER_NAME,
    .dev       = &device_desc,
    .strings   = dev_strings,
    .bind      = composite_bind,
    .unbind    = composite_unbind,
    .max_speed = USB_SPEED_HIGH,
};

module_usb_composite_driver(bulk_driver);
