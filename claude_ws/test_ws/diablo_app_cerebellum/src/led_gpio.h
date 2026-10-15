

#ifndef LED_GPIO_H_
#define LED_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

enum led_type_t {
    LED_READY = 0,
    LED_LOCOMOTION = 1,
    LED_FAULT = 2,
    LED_DEBUG = 3,
};

enum led_state_t {
    LED_OFF = 0,
    LED_ON = 1,
};

int led_gpio_init();
void led_gpio_set(enum led_type_t i, enum led_state_t state);
void led_gpio_toggle(enum led_type_t i);
void led_gpio_set_indicator(enum led_type_t i);
void led_gpio_toggle_indicator();


#ifdef __cplusplus
}
#endif

#endif  // LED_GPIO_H_
