

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

#include "api_acm.h"
#include "api_canfd.h"
#include "api_protocol.h"

LOG_MODULE_REGISTER(api_acm, LOG_LEVEL_INF);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];

struct ring_buf ringbuf;

#define ACM_RX_BUF_SIZE CONFIG_USB_CDC_ACM_RINGBUF_SIZE
#define ACM_TX_BUF_SIZE CONFIG_USB_CDC_ACM_RINGBUF_SIZE

struct acm_can_data_t {
	uint8_t acm_rx_buf[ACM_RX_BUF_SIZE];
	uint8_t acm_tx_buf[ACM_TX_BUF_SIZE];
	struct ring_buf acm_rx_ringbuf;
	struct ring_buf acm_tx_ringbuf;

	const struct device *usbacm;
};

static struct acm_can_data_t _acm_can_data;
static struct api_ops_t api_acm_ops;

static const struct device *const usbacm = DEVICE_DT_GET(DT_CHOSEN(zephyr_api_usbacm));

static void interrupt_handler(const struct device *dev, void *user_data)
{
	struct acm_can_data_t *data = (struct acm_can_data_t *)user_data;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
#if TODO
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
#endif
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t *buffer;
			int rb_len, send_len;

			rb_len = ring_buf_get_claim(&data->acm_tx_ringbuf, &buffer, ACM_TX_BUF_SIZE);
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			ring_buf_get_finish(&data->acm_tx_ringbuf, rb_len);

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

int api_acm_init(struct robot_t *robot)
{
    int ret;

	if (!device_is_ready(usbacm)) {
		LOG_ERR("CDC ACM device not ready");
		return -1;
	}

    ret = usb_enable(0);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return ret;
	}

	struct acm_can_data_t *data = &_acm_can_data;

	ring_buf_init(&data->acm_rx_ringbuf, ACM_RX_BUF_SIZE, data->acm_rx_buf);
	ring_buf_init(&data->acm_tx_ringbuf, ACM_TX_BUF_SIZE, data->acm_tx_buf);

	data->usbacm = usbacm;
	uart_irq_callback_user_data_set(data->usbacm, interrupt_handler, (void *)data);

	robot->acm_data = (void *)data;

	robot->api_ops = &api_acm_ops;

    uart_irq_rx_enable(usbacm);

	return 0;
}

int api_acm_send_packet(struct acm_can_data_t *data, void *packet, size_t pkt_len)
{
	int ret = 0;
	uint8_t *buffer;
	uint32_t size, length;

    if (ring_buf_is_empty(&data->acm_tx_ringbuf)) {
        ring_buf_reset(&data->acm_tx_ringbuf);
    }

	length = ACM_HDR_LEN + pkt_len;

    size = ring_buf_put_claim(&data->acm_tx_ringbuf, &buffer, length);
    if (size < length) {
        ring_buf_put_finish(&data->acm_tx_ringbuf, 0);
		return;
	}

	struct acm_pkt_t *pkt = (struct acm_pkt *)buffer;

	pkt->tag = ACM_TAG;
    pkt->length = length;
    pkt->checksum = crc16_ansi((void *)packet, length - ACM_HDR_LEN);
    pkt->type = ACM_TYPE_CAM;

	memcpy((void *)(pkt + 1), packet, pkt_len);

    ret = ring_buf_put_finish(&data->acm_tx_ringbuf, size);
    if (ret != 0) {
        LOG_ERR("acm_tx_ringbuf put finish failed");
    }

	uart_irq_tx_enable(data->usbacm);
}

/** upload(send) imu/ahrs data */
void api_acm_upload_imu_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
	int ret = 0;
	uint8_t data_len = 0;
	uint32_t size, length;
	struct acm_can_data_t *data = (struct acm_can_data_t *)robot->acm_data;

    struct acm_can_pkt_t _can_pkt;
	struct acm_can_pkt_t *can_pkt = &_can_pkt;
	struct api_imu_data_t *api_imu_data = &can_pkt->data;

    api_imu_data->timestamp = robot->timestamp;
    api_imu_data->accel[0] = robot->accel[0];
    api_imu_data->accel[1] = robot->accel[1];
    api_imu_data->accel[2] = robot->accel[2];
    api_imu_data->gyro[0] = robot->gyro[0];
    api_imu_data->gyro[1] = robot->gyro[1];
    api_imu_data->gyro[2] = robot->gyro[2];
    api_imu_data->quaternion[0] = robot->quaternion[0];
    api_imu_data->quaternion[1] = robot->quaternion[1];
    api_imu_data->quaternion[2] = robot->quaternion[2];
    api_imu_data->quaternion[3] = robot->quaternion[3];

	can_pkt->id = CAN_ID_IMU1;
    can_pkt->dlc = can_bytes_to_dlc(sizeof(struct api_imu_data_t));
    can_pkt->flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	data_len = can_dlc_to_bytes(can_pkt->dlc);
	data_len = data_len < 8 ? 8 : data_len;
	length = ACM_CAN_HDR_LEN + data_len;

	ret = api_acm_send_packet(data, can_pkt, length);
	if (ret != 0) {

	}

	return;
}

/** upload(send) motor out/in data */
void api_acm_upload_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{
    // copy wrapper data to api_state
    struct motor_group_t *group = 0;

    for (int i = LEFT; i <= RIGHT; i++) {
        int idx = i * 4;
        struct motor_t *motor = 0;
        struct api_motor_in_t *api_in = 0;
        struct api_motor_out_t *api_out = 0;

        const struct wrapper_robot_t *wrapper = robot->priv;

        group = wrap_get_motor_group(wrapper, i);

        for (int i = M_ABAD; i <= M_ANKLE; i++) {
            motor = &group->motors[i];
            api_in = &api_state->in[idx + i];
            api_out = &api_state->out[idx + i];

            api_in->timestamp = motor->in.timestamp;
            api_in->position = motor->in.position;
            api_in->velocity = motor->in.velocity;
            api_in->torque = motor->in.torque;

            api_out->timestamp = motor->out.timestamp;
            api_out->position = motor->out.position;
            api_out->kp = motor->out.kp;
            api_out->velocity = motor->out.velocity;
            api_out->kd = motor->out.kd;
            api_out->torque = motor->out.torque;
        }
    }

    // send api_state to bus
    int ret = 0;
    size_t dsize = 0;
	uint8_t data_len = 0;
	uint32_t size, length;
    // struct can_frame frame;
    struct acm_can_pkt_t _can_pkt;
	struct acm_can_pkt_t *can_pkt = &_can_pkt;
	// struct api_imu_data_t *api_imu_data = &can_pkt->data;

	struct acm_can_data_t *data = (struct acm_can_data_t *)robot->acm_data;

    dsize = sizeof(struct api_motor_in_t) * 4;

    for (int i = 0; i < API_MAX_MOTORS / 4; i++) {
        can_pkt->id = CAN_ID_MOTOR_IN + i;
        can_pkt->dlc = can_bytes_to_dlc(dsize);
        can_pkt->flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

        memcpy(can_pkt->data, api_state->in + i * 4, dsize);

		data_len = can_dlc_to_bytes(can_pkt->dlc);
		data_len = data_len < 8 ? 8 : data_len;
		length = ACM_CAN_HDR_LEN + data_len;

		ret = api_acm_send_packet(data, can_pkt, length);
		if (ret != 0) {

		}
    }

    dsize = sizeof(struct api_motor_out_t) * 2;

    for (int i = 0; i < API_MAX_MOTORS / 2; i++) {
        can_pkt->id = CAN_ID_MOTOR_OUT + i;
        can_pkt->dlc = can_bytes_to_dlc(dsize);
        can_pkt->flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

        memcpy(can_pkt->data, api_state->out + i * 2, dsize);

		data_len = can_dlc_to_bytes(can_pkt->dlc);
		data_len = data_len < 8 ? 8 : data_len;
		length = ACM_CAN_HDR_LEN + data_len;

		ret = api_acm_send_packet(data, can_pkt, length);
		if (ret != 0) {

		}
    }
}

/** swap motor in/out data, recv motor out(desired) data, send motor in(real) data */
void api_acm_swap_motor_data(struct robot_t *robot, struct api_state_t *api_state, const float dt)
{

}

static struct api_ops_t api_acm_ops = {
    .upload_imu_data = api_acm_upload_imu_data,
    .upload_motor_data = api_acm_upload_motor_data,
    .swap_motor_data = api_acm_swap_motor_data,
};
