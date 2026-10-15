

#include "led_gpio.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);


#if defined(CONFIG_BOARD_DIRECTDRIVETECH_APOLLO)

static struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
};

#elif defined(CONFIG_BOARD_DIRECTDRIVETECH_APOLLO_EVK)

static struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios),
};

#elif defined(CONFIG_BOARD_DIRECTDRIVETECH_DIABLO_ATHENA)

static struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
};

#else

#error "not support new board"

#endif

#define LED_NUM ARRAY_SIZE(leds)

int led_gpio_init()
{
    int ret = 0;
    const struct gpio_dt_spec *led = 0;

    for (int i = 0; i < LED_NUM; i++) {
        led = &leds[i];

        if (device_is_ready(led->port) != true) {
            LOG_ERR("led%d device not ready", i);
            return -ENODEV; 
        }

        ret = gpio_pin_configure_dt(led, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            return ret;
        }

        ret = gpio_pin_set_dt(led, LED_OFF);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

void led_gpio_set(enum led_type_t i, enum led_state_t state)
{
    if (i < 0 || i >= LED_NUM)
        return;

    gpio_pin_set_dt(&leds[i], state);
}

void led_gpio_toggle(enum led_type_t i)
{
    if (i < 0 || i >= LED_NUM)
        return;
    gpio_pin_toggle_dt(&leds[i]);
}

static enum led_type_t indicator = LED_READY;

void led_gpio_set_indicator(enum led_type_t i)
{
    led_gpio_set(indicator, LED_OFF);

    indicator = i;
}

void led_gpio_toggle_indicator()
{
    led_gpio_toggle(indicator);
}
