/*
 * device_fs.c — Jetson Device 侧 (functionfs)
 *
 * 编译: gcc -O2 -o device_fs device_fs.c -lpthread
 * 运行: sudo ./device_fs /mnt/ffs_stress
 *
 * 流程:
 *   1. Jetson 上先跑 gadget_setup.sh
 *   2. 再跑 ./device_fs /mnt/ffs_stress (这会阻塞等待 Host 枚举)
 *   3. 另开一个终端: echo ci_hdrc.0 > /sys/kernel/config/usb_gadget/g_stress/UDC
 *      注意 ci_hdrc.0 要换成实际的 UDC 名称
 *   4. Host (RK3588) 上跑 host_libusb
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdatomic.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>

#include <sys/poll.h>
#include <sys/types.h>

#include <linux/usb/ch9.h>
#include <linux/usb/functionfs.h>

/* ========= 配置 ========= */
#define MAX_PAYLOAD     (16 * 1024)   /* 单次读写最大字节 */
#define STATS_INTERVAL  1             /* 打印间隔(秒) */

/* ========= 全局状态 ========= */
static atomic_int g_running = 1;
static atomic_ulong g_tx_bytes = 0;
static atomic_ulong g_rx_bytes = 0;

/* ========= USB 描述符 ========= */

/* 这里我们定义 1 个 Interface + 2 个 Bulk 端点:
 *   EP 0x01: Bulk OUT (Host→Device)
 *   EP 0x82: Bulk IN  (Device→Host)
 * 注意: functionfs 会按描述符顺序创建 ep1(OUT), ep2(IN)
 */
static const struct {
    struct usb_functionfs_descs_head_v2 header;
    __le32 fs_count;
    __le32 hs_count;
    __le32 ss_count;

    /* Full Speed 描述符组 */
    struct usb_interface_descriptor fs_intf;
    struct usb_endpoint_descriptor fs_ep_out;
    struct usb_endpoint_descriptor fs_ep_in;

    /* High Speed 描述符组 */
    struct usb_interface_descriptor hs_intf;
    struct usb_endpoint_descriptor hs_ep_out;
    struct usb_endpoint_descriptor hs_ep_in;

    /* Super Speed 描述符组 (兼容) */
    struct usb_interface_descriptor ss_intf;
    struct usb_endpoint_descriptor ss_ep_out;
    struct usb_endpoint_descriptor ss_ep_in;
} descriptors = {
    .header = {
        .magic = htole32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2),
        .length = htole32(sizeof(descriptors)),
        .flags = htole32(FUNCTIONFS_HAS_FS_DESC |
                         FUNCTIONFS_HAS_HS_DESC |
                         FUNCTIONFS_HAS_SS_DESC)
    },
    /* FS 计数: 1 interface, 2 endpoints */
    .fs_count = htole32(1),
    .hs_count = htole32(1),
    .ss_count = htole32(1),

    /* --- Full Speed (12 Mbps) --- */
    .fs_intf = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0xFF,
        .bInterfaceSubClass = 0x42,
        .bInterfaceProtocol = 0x01,
        .iInterface = 0,
    },
    .fs_ep_out = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,  /* OUT */
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = htole16(64),  /* FS 最大 64 */
        .bInterval = 0,
    },
    .fs_ep_in = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,  /* IN */
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = htole16(64),
        .bInterval = 0,
    },

    /* --- High Speed (480 Mbps) --- */
    .hs_intf = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0xFF,
        .bInterfaceSubClass = 0x42,
        .bInterfaceProtocol = 0x01,
        .iInterface = 0,
    },
    .hs_ep_out = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = htole16(512),  /* HS 最大 512 */
        .bInterval = 0,
    },
    .hs_ep_in = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = htole16(512),
        .bInterval = 0,
    },

    /* --- Super Speed (5 Gbps) --- */
    .ss_intf = {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0xFF,
        .bInterfaceSubClass = 0x42,
        .bInterfaceProtocol = 0x01,
        .iInterface = 0,
    },
    .ss_ep_out = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = htole16(1024),
        .bInterval = 0,
    },
    .ss_ep_in = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = htole16(1024),
        .bInterval = 0,
    },
};

/* ========= 字符串 ========= */

static const struct {
    struct usb_functionfs_strings_head header;
    __le16 str_manufacturer;
    __le16 str_product;
    __le16 str_serial;
    __le16 str_config;
    __le16 str_interface;
} strings = {
    .header = {
        .magic = htole32(FUNCTIONFS_STRINGS_MAGIC),
        .length = htole32(sizeof(strings)),
        .str_count = htole32(1),
        .lang[0] = htole16(0x0409),
    },
};

/* ========= 发送线程 (往 Host 发: IN EP write) ========= */
static void *send_thread_fn(void *arg)
{
    int fd = *(int *)arg;
    char buf[MAX_PAYLOAD] __attribute__((aligned(16)));

    while (atomic_load(&g_running)) {
        ssize_t n = write(fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            if (atomic_load(&g_running)) perror("send write");
            break;
        }
        atomic_fetch_add(&g_tx_bytes, n);
    }
    return NULL;
}

/* ========= 接收线程 (从 Host 收: OUT EP read) ========= */
static void *recv_thread_fn(void *arg)
{
    int fd = *(int *)arg;
    char buf[MAX_PAYLOAD] __attribute__((aligned(16)));

    while (atomic_load(&g_running)) {
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            if (atomic_load(&g_running)) perror("recv read");
            break;
        }
        atomic_fetch_add(&g_rx_bytes, n);
    }
    return NULL;
}

/* ========= 统计线程 ========= */
static void *stats_thread_fn(void *arg)
{
    (void)arg;
    unsigned long prev_tx = 0, prev_rx = 0;

    while (atomic_load(&g_running)) {
        sleep(STATS_INTERVAL);
        unsigned long tx = atomic_load(&g_tx_bytes);
        unsigned long rx = atomic_load(&g_rx_bytes);
        unsigned long tx_delta = tx - prev_tx;
        unsigned long rx_delta = rx - prev_rx;

        printf("[Device] TX: %9lu B/s (%6.1f Mb/s)  "
               "RX: %9lu B/s (%6.1f Mb/s)  "
               "Total: TX=%lu MB  RX=%lu MB\n",
               tx_delta, (double)tx_delta * 8 / 1024 / 1024,
               rx_delta, (double)rx_delta * 8 / 1024 / 1024,
               tx / 1024 / 1024, rx / 1024 / 1024);
        fflush(stdout);
        prev_tx = tx;
        prev_rx = rx;
    }
    return NULL;
}

static void handle_sigint(int sig)
{
    (void)sig;
    atomic_store(&g_running, 0);
}

int main(int argc, char *argv[])
{
    const char *mount_path;
    if (argc >= 2)
        mount_path = argv[1];
    else
        mount_path = "/mnt/ffs_stress";

    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    printf("Device FS — 打开 functionfs: %s\n", mount_path);

    /* 打开 ep0 */
    char ep0_path[256];
    snprintf(ep0_path, sizeof(ep0_path), "%s/ep0", mount_path);
    int ep0_fd = open(ep0_path, O_RDWR);
    if (ep0_fd < 0) { perror("open ep0"); return 1; }

    /* 写描述符 */
    if (write(ep0_fd, &descriptors, sizeof(descriptors)) < 0)
    { perror("write descriptors"); close(ep0_fd); return 1; }

    /* 写字符串 */
    if (write(ep0_fd, &strings, sizeof(strings)) < 0)
    { perror("write strings"); close(ep0_fd); return 1; }

    printf("描述符已写入，等待 Host 枚举...\n");

    /* 等待 Host 枚举 (ep0 可读 = SET_CONFIGURATION) */
    struct pollfd pfd = { .fd = ep0_fd, .events = POLLIN };
    int ret = poll(&pfd, 1, 30000);
    if (ret <= 0) {
        fprintf(stderr, "poll ep0 timeout/error\n");
        close(ep0_fd);
        return 1;
    }

    printf("Host 已枚举！打开端点...\n");

    /* 打开端点文件: ep1 = OUT, ep2 = IN */
    char ep_path[256];
    snprintf(ep_path, sizeof(ep_path), "%s/ep1", mount_path);
    int ep_out_fd = open(ep_path, O_RDWR);
    if (ep_out_fd < 0) { perror("open ep1"); close(ep0_fd); return 1; }

    snprintf(ep_path, sizeof(ep_path), "%s/ep2", mount_path);
    int ep_in_fd = open(ep_path, O_RDWR);
    if (ep_in_fd < 0) { perror("open ep2"); close(ep_out_fd); close(ep0_fd); return 1; }

    printf("启动双向压力测试...\n");

    pthread_t send_thr, recv_thr, stats_thr;
    pthread_create(&send_thr, NULL, send_thread_fn, &ep_in_fd);
    pthread_create(&recv_thr, NULL, recv_thread_fn, &ep_out_fd);
    pthread_create(&stats_thr, NULL, stats_thread_fn, NULL);

    pthread_join(send_thr, NULL);
    pthread_join(recv_thr, NULL);
    pthread_join(stats_thr, NULL);

    close(ep_in_fd);
    close(ep_out_fd);
    close(ep0_fd);

    unsigned long tx = atomic_load(&g_tx_bytes);
    unsigned long rx = atomic_load(&g_rx_bytes);
    printf("\n=== 结束 === TX=%lu MB  RX=%lu MB\n",
           tx / 1024 / 1024, rx / 1024 / 1024);
    return 0;
}
