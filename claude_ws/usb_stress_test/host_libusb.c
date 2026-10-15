/*
 * host_libusb.c — RK3588 Host 侧 (libusb)
 *
 * 编译: gcc -O2 -o host_libusb host_libusb.c -lpthread -lusb-1.0
 * 运行: sudo ./host_libusb
 *
 * 依赖: apt install libusb-1.0-0-dev
 *
 * 自动检测 Bulk 端点地址
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include <libusb-1.0/libusb.h>

/* ========= 配置 ========= */
#define VENDOR_ID   0x1d6b
#define PRODUCT_ID  0x0104
#define MAX_PAYLOAD  (16 * 1024)
#define TIMEOUT_MS  1000
#define STATS_INTERVAL 1

/* ========= 全局 ========= */
static atomic_int g_running = 1;
static atomic_ulong g_tx_bytes = 0;
static atomic_ulong g_rx_bytes = 0;
static libusb_device_handle *g_dev = NULL;
static unsigned char g_ep_out = 0;
static unsigned char g_ep_in  = 0;

/* ========= 自动发现 Bulk 端点 ========= */
static int find_endpoints(libusb_device *dev)
{
    struct libusb_config_descriptor *config = NULL;
    int ret = libusb_get_active_config_descriptor(dev, &config);
    if (ret < 0) {
        fprintf(stderr, "get_config: %s\n", libusb_error_name(ret));
        return -1;
    }

    for (int i = 0; i < config->bNumInterfaces; i++) {
        for (int j = 0; j < config->interface[i].num_altsetting; j++) {
            const struct libusb_interface_descriptor *intf =
                &config->interface[i].altsetting[j];
            for (int k = 0; k < intf->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor *ep =
                    &intf->endpoint[k];
                if ((ep->bmAttributes & 0x03) == LIBUSB_TRANSFER_TYPE_BULK) {
                    if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN)
                        g_ep_in = ep->bEndpointAddress;
                    else
                        g_ep_out = ep->bEndpointAddress;
                }
            }
        }
    }

    libusb_free_config_descriptor(config);

    if (!g_ep_out || !g_ep_in) {
        fprintf(stderr, "未找到 Bulk endpoints\n");
        return -1;
    }

    printf("发现 Bulk: OUT=0x%02x  IN=0x%02x\n", g_ep_out, g_ep_in);
    return 0;
}

/* ========= 发送线程 ========= */
static void *send_thread_fn(void *arg)
{
    (void)arg;
    unsigned char buf[MAX_PAYLOAD] __attribute__((aligned(16)));
    int transferred;

    while (atomic_load(&g_running)) {
        int ret = libusb_bulk_transfer(g_dev, g_ep_out, buf, sizeof(buf),
                                       &transferred, TIMEOUT_MS);
        if (ret == LIBUSB_ERROR_TIMEOUT) continue;
        if (ret < 0) {
            if (atomic_load(&g_running))
                fprintf(stderr, "send: %s\n", libusb_error_name(ret));
            break;
        }
        atomic_fetch_add(&g_tx_bytes, transferred);
    }
    return NULL;
}

/* ========= 接收线程 ========= */
static void *recv_thread_fn(void *arg)
{
    (void)arg;
    unsigned char buf[MAX_PAYLOAD] __attribute__((aligned(16)));
    int transferred;

    while (atomic_load(&g_running)) {
        int ret = libusb_bulk_transfer(g_dev, g_ep_in, buf, sizeof(buf),
                                       &transferred, TIMEOUT_MS);
        if (ret == LIBUSB_ERROR_TIMEOUT) continue;
        if (ret < 0) {
            if (atomic_load(&g_running))
                fprintf(stderr, "recv: %s\n", libusb_error_name(ret));
            break;
        }
        atomic_fetch_add(&g_rx_bytes, transferred);
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

        printf("[Host] TX: %9lu B/s (%6.1f Mb/s)  "
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

int main(void)
{
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    libusb_context *ctx = NULL;
    int ret = libusb_init(&ctx);
    if (ret < 0) {
        fprintf(stderr, "libusb_init: %s\n", libusb_error_name(ret));
        return 1;
    }

    g_dev = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!g_dev) {
        fprintf(stderr, "未找到设备 %04x:%04x\n"
                "Jetson 上先: sudo insmod g_bulk.ko\n",
                VENDOR_ID, PRODUCT_ID);
        libusb_exit(ctx);
        return 1;
    }
    printf("已连接设备\n");

    libusb_device *dev = libusb_get_device(g_dev);
    if (find_endpoints(dev) < 0) {
        libusb_close(g_dev);
        libusb_exit(ctx);
        return 1;
    }

    if (libusb_kernel_driver_active(g_dev, 0) == 1) {
        libusb_detach_kernel_driver(g_dev, 0);
        printf("已 detach 内核驱动\n");
    }

    ret = libusb_claim_interface(g_dev, 0);
    if (ret < 0) {
        fprintf(stderr, "claim_interface: %s\n", libusb_error_name(ret));
        libusb_close(g_dev);
        libusb_exit(ctx);
        return 1;
    }
    printf("Interface 0 claimed\n");
    printf("启动双向压力测试...\n");

    pthread_t send_thr, recv_thr, stats_thr;
    pthread_create(&send_thr, NULL, send_thread_fn, NULL);
    pthread_create(&recv_thr, NULL, recv_thread_fn, NULL);
    pthread_create(&stats_thr, NULL, stats_thread_fn, NULL);

    pthread_join(send_thr, NULL);
    pthread_join(recv_thr, NULL);
    pthread_join(stats_thr, NULL);

    libusb_release_interface(g_dev, 0);
    libusb_close(g_dev);
    libusb_exit(ctx);

    unsigned long tx = atomic_load(&g_tx_bytes);
    unsigned long rx = atomic_load(&g_rx_bytes);
    printf("\n=== 结束 === TX=%lu MB  RX=%lu MB\n",
           tx / 1024 / 1024, rx / 1024 / 1024);
    return 0;
}
