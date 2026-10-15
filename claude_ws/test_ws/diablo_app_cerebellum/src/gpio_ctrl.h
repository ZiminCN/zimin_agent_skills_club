#ifndef GPIO_CTRL_H_
#define GPIO_CTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

enum gpio_type_t {
    PIN_D13 = 0,
    PIN_D14 = 1,
    PIN_B14 = 2,
    LDO_3V3 = 3,
    LDO_1V8 = 4,
};

enum robot_model_t {
    ROBOT_MODEL_FRONT,
    ROBOT_MODEL_REAR,
};

extern int gpio_ctrl_dvt3_mark;

int gpio_ctrl_init();
int gpio_ctrl_update();
int gpio_get_robot_model();

#ifdef __cplusplus
}
#endif

#endif // GPIO_CTRL_H_
