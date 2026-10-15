#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>

static volatile int running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    running = 0;
}

static void print_canfd_frame(const struct canfd_frame *frame)
{
    printf("%08X  ", frame->can_id);

    if (frame->can_id & CAN_EFF_FLAG)
        printf("EXT  ");
    else
        printf("STD  ");

    if (frame->can_id & CAN_RTR_FLAG) {
        printf("RTR\n");
        return;
    }

    if (frame->flags & CANFD_BRS)
        printf("BRS  ");
    if (frame->flags & CANFD_ESI)
        printf("ESI  ");

    printf("[%d]  ", frame->len);

    for (int i = 0; i < frame->len; i++)
        printf("%02X ", frame->data[i]);

    printf("\n");
}

int main(int argc, char *argv[])
{
    const char *ifname = "can0";
    if (argc > 1)
        ifname = argv[1];

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    int canfd_on = 1;
    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on)) < 0) {
        perror("setsockopt CAN_RAW_FD_FRAMES");
        close(sock);
        return 1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close(sock);
        return 1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return 1;
    }

    printf("Listening on %s (CAN FD enabled), Ctrl+C to stop...\n", ifname);
    fflush(stdout);

    fd_set rdfs;
    struct canfd_frame frame;

    while (running) {
        FD_ZERO(&rdfs);
        FD_SET(sock, &rdfs);

        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };

        int ret = select(sock + 1, &rdfs, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            perror("select");
            break;
        }
        if (ret == 0)
            continue;

        ssize_t nbytes = read(sock, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (errno == EINTR)
                continue;
            perror("read");
            break;
        }

        switch (nbytes) {
        case CAN_MTU:
            printf("CAN   ");
            break;
        case CANFD_MTU:
            printf("CANFD ");
            break;
        default:
            printf("?%zd   ", nbytes);
            break;
        }

        print_canfd_frame(&frame);
        fflush(stdout);
    }

    close(sock);
    printf("Done.\n");
    return 0;
}
