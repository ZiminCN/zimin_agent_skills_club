

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>

#include "rc_sbus.h"

LOG_MODULE_REGISTER(rc_sbus, LOG_LEVEL_INF);

#define SBUS_START 0x0F
#define SBUS_END 0x00

#define SBUS_BUF_SIZE 25
#define SBUS_DMA_BUF_SIZE 32
#define SBUS_DMA_BUF_CNT 3
#define SBUS_IDLE_TIMEOUT 1000 // microsecond
// SBUS messages take 3ms to communicate.
// Typical repeat intervals are: 14ms (analog mode), 7ms (high speed mode)

static uint8_t rx_count = 0;
static uint8_t align_count = 0;
static uint8_t current_dma_buf = 0;
static uint8_t dma_buffer[SBUS_DMA_BUF_SIZE * SBUS_DMA_BUF_CNT];

static sbus_frame_t g_sbus_frame = {0};

const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(sbus0));

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    struct robot_t *robot = (struct robot_t *)user_data;

    struct uart_event_rx *rx = NULL;
    struct uart_event_rx_buf *rx_buf = NULL;
    struct uart_event_rx_stop *rx_stop = NULL;

    uint8_t *buf;
    uint32_t len;
    uint32_t used;

    switch (evt->type) {
    case UART_RX_RDY:
        rx = &evt->data.rx;
        len = rx->len;
        buf = rx->buf + rx->offset;

        if (len == 0)
            break;

        if (len != SBUS_BUF_SIZE || buf[0] != SBUS_START || buf[SBUS_BUF_SIZE - 1] != SBUS_END)
        {
            // drop half-packet data, restart dma try align
            uart_rx_disable(dev);
            align_count += 1;
            if (align_count > 10) {
                LOG_ERR("sbus dma align failed, sbus will stop");
            }
            LOG_DBG("sbus buffer not aligned(%d), try again", len);
            LOG_HEXDUMP_DBG(buf, len, "sbus raw");
        }

        align_count = 0;

        memcpy(&g_sbus_frame, buf, SBUS_BUF_SIZE);
        
        if (robot->rc_connected == 0 && rx_count > 250) {
            robot->rc_connected = 1;
        } else {
            rx_count += 1;
        }
        break;

    case UART_RX_BUF_REQUEST:
        current_dma_buf += 1;
        current_dma_buf %= SBUS_DMA_BUF_CNT;
        buf = dma_buffer + SBUS_DMA_BUF_SIZE * current_dma_buf;
        uart_rx_buf_rsp(dev, buf, SBUS_BUF_SIZE);
        break;

    case UART_RX_BUF_RELEASED:
        rx_buf = &evt->data.rx_buf;
        break;

    case UART_RX_DISABLED:
        // restart dma, TODO: add restart flag
        if (align_count > 10)
            break;

        current_dma_buf += 1;
        current_dma_buf %= SBUS_DMA_BUF_CNT;
        buf = dma_buffer + SBUS_DMA_BUF_SIZE * current_dma_buf;
        uart_rx_enable(dev, buf, SBUS_BUF_SIZE, SBUS_IDLE_TIMEOUT);
        LOG_DBG("Restart RX");
        break;

    case UART_RX_STOPPED:
        rx_stop = &evt->data.rx_stop;
        break;

    default:
        break;
    }
}

int rc_sbus_init(struct robot_t *robot)
{
    int ret = 0;
    struct uart_config cfg;

    // sbus 100kbps, 8E2
    cfg.baudrate = 100000;
    cfg.data_bits = UART_CFG_DATA_BITS_8;
    cfg.parity = UART_CFG_PARITY_EVEN;
    cfg.stop_bits = UART_CFG_STOP_BITS_2;
    cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

    ret = uart_configure(uart_dev, &cfg);
    if (ret != 0) {
      LOG_ERR("sbus configure failed");
    }

    ret = uart_callback_set(uart_dev, uart_callback, (void *)robot);
    if (ret != 0) {
      LOG_ERR("uart set callback failed: %d", ret);
    }

    ret = uart_rx_enable(uart_dev, dma_buffer, SBUS_BUF_SIZE, SBUS_IDLE_TIMEOUT);
    if (ret != 0) {
      LOG_ERR("uart rx enable failed: %d", ret);
    }
}

// timeout trigger frame_lost
// lost 170 frames (1sec) trigger failsafe

typedef struct rc_sbus_state_t {
    uint16_t ch1_median;
    uint16_t ch2_median;
    uint16_t ch3_median;
    uint16_t ch4_median;
    bool inited;
    uint32_t count;
} rc_sbus_state_t;

static rc_sbus_state_t g_sbus_state = {1000, 1000, 1000, 1000, false, 0};

static inline uint8_t decode_jump(uint16_t input)
{
    uint8_t out;

    out = input == 200 ? 0 : 1;

    return out;
}

static inline float decode_height(uint16_t input)
{
    float out;

    out = (float)(input - 200) / (1800 - 200);
    out = out <= 1.0 ? out : 1.0; // out of range

    return out;
}

static inline float decode_axis(uint16_t input, uint16_t median)
{
    float out;

    out = (float)(input - median) / median / 0.8;
    out = out <= 1.0 ? out : 1.0; // out of range
    out = fabs(out) < 0.01 ? 0 : out; // deadzone

    return out;
}

enum rc_state_t {
    RC_MODE_0,
    RC_MODE_1,
    RC_MODE_2,
};

static inline enum rc_state_t decode_state(uint16_t input)
{
    enum rc_state_t state = RC_MODE_1;

    switch (input) {
      case 1800:
        state = RC_MODE_0;
        break;
      case 1000:
        state = RC_MODE_1;
        break;
      case 200:
        state = RC_MODE_2;
        break;
      default:
        state = RC_MODE_1;
    }

    return state;
}

static bool rc_sbus_parse(sbus_frame_t *frame, input_t *input)
{
    if (frame == NULL || input == NULL)
        return false;

    memset(input, 1, sizeof(input_t));

    if (frame->start != SBUS_START || frame->end != SBUS_END) {
        return false;
    }

    if (frame->frame_lost || frame->failsafe) {
        g_sbus_state.inited = false;
        g_sbus_state.count = 0;
    }

    if (g_sbus_state.inited == false) {
        // record zero point
        if (abs(frame->ch1 - 1000) < 100 && abs(frame->ch2 - 1000) < 100 &&
            abs(frame->ch3 - 1000) < 100 && abs(frame->ch4 - 1000) < 100) {
            g_sbus_state.count += 1;
            g_sbus_state.ch1_median = (g_sbus_state.ch1_median + frame->ch1) / 2;
            g_sbus_state.ch2_median = (g_sbus_state.ch2_median + frame->ch2) / 2;
            g_sbus_state.ch3_median = (g_sbus_state.ch3_median + frame->ch3) / 2;
            g_sbus_state.ch4_median = (g_sbus_state.ch4_median + frame->ch4) / 2;

            if (g_sbus_state.count > 480) {
                g_sbus_state.inited = true;
                g_sbus_state.count = 0;
            }
        }
        // input->lost = true;
    } else {
        // input output
        input->yaw = decode_axis(frame->ch1, g_sbus_state.ch1_median);
        input->forward = -decode_axis(frame->ch2, g_sbus_state.ch2_median); // ch2 is rev
        input->pitch = decode_axis(frame->ch3, g_sbus_state.ch3_median);
        input->roll = decode_axis(frame->ch4, g_sbus_state.ch4_median);

        // input->left = decode_state(frame->ch7);
        // input->right = decode_state(frame->ch5);

        input->height = decode_height(frame->ch8);
        // input->jump = decode_jump(frame->ch6);

        // input->lost = false;

        g_sbus_state.count += 1;
        input->update_count = g_sbus_state.count;
    }

    // return input->lost;
    return false;
}

bool rc_update_input(input_t *input)
{
    rc_sbus_parse(&g_sbus_frame, input);
    // return input->lost;
    return false;
}
