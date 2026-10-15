

#include "gpio_ctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio, LOG_LEVEL_INF);

int gpio_ctrl_dvt3_mark = 0;

#if defined(CONFIG_BOARD_DIRECTDRIVETECH_APOLLO)

static struct gpio_dt_spec gpios[] = {
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin_d13_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin_d14_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin_b14_gpios)};

#elif defined(CONFIG_BOARD_DIRECTDRIVETECH_APOLLO_EVK)

static struct gpio_dt_spec gpios[] = {
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin_d13_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin_d14_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin_b14_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pwr_3v3_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pwr_1v8_gpios)};

#else

#endif

#define GPIO_NUM ARRAY_SIZE(gpios)

static enum robot_model_t robot_model = ROBOT_MODEL_FRONT;

int gpio_ctrl_init()
{
#if defined(CONFIG_BOARD_DIRECTDRIVETECH_APOLLO_EVK)

    const struct gpio_dt_spec *ldo_3v3 = &gpios[LDO_3V3];

    gpio_pin_configure_dt(ldo_3v3, GPIO_INPUT);

    if (gpio_pin_get_dt(ldo_3v3) != 0) {
        LOG_ERR("Hardware Version BUG");
    }

#endif

#if defined(CONFIG_BOARD_DIRECTDRIVETECH_APOLLO) || \
    defined(CONFIG_BOARD_DIRECTDRIVETECH_APOLLO_EVK)
    const struct gpio_dt_spec *pin_d13 = &gpios[PIN_D13];
    const struct gpio_dt_spec *pin_b14 = &gpios[PIN_B14];
    const struct gpio_dt_spec *pin_d14 = &gpios[PIN_D14];

    gpio_pin_configure_dt(pin_d13, GPIO_INPUT);
    gpio_pin_configure_dt(pin_b14, GPIO_INPUT);
    gpio_pin_configure_dt(pin_d14, GPIO_INPUT);

    k_msleep(10);

    if (gpio_pin_get_dt(pin_d13) == 1) {
        robot_model = ROBOT_MODEL_REAR;
    }
    if (gpio_pin_get_dt(pin_b14) == 1) {
        // DVT3 Mark, R3 0ohm to GND
        gpio_ctrl_dvt3_mark = 1;
    }
#endif

#if defined(CONFIG_BOARD_DIRECTDRIVETECH_DIABLO_ATHENA)
    motor_model = MODEL_M1502_M1502;
#endif

    return 0;
}

int gpio_ctrl_update()
{
    const struct gpio_dt_spec *pin_d13 = &gpios[PIN_D13];

    gpio_pin_configure_dt(pin_d13, GPIO_INPUT);

    if (gpio_pin_get_dt(pin_d13) == 1) {
        robot_model = ROBOT_MODEL_REAR;
    } else {
        robot_model = ROBOT_MODEL_FRONT;
    }

    return robot_model;
}

int gpio_get_robot_model()
{
    // force FRONT
    // return ROBOT_MODEL_FRONT;

    return robot_model;
}
