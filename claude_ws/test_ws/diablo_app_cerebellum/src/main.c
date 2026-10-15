

#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include "ahrs.h"
#include "led_gpio.h"
#include "gpio_ctrl.h"
#include "api_acm.h"
#include "api_canfd.h"
#include "rc_sbus.h"
#include "wrapper.h"
#include "robot.h"
#ifdef CONFIG_APP_ABUP_FOTA		
#include "abup_fota.h"
#include "can-fd.h"
#endif


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static inline int hardware_init() {

    gpio_ctrl_init();

    led_gpio_init();

    return 0;
}

int main(void)
{
    hardware_init();

#ifdef CONFIG_APP_ABUP_FOTA		
    canFd_driverInit();
#endif

    enum robot_model_t robot_model = gpio_get_robot_model();
    if (robot_model == ROBOT_MODEL_FRONT)
        LOG_INF("Robot Model: Normal/Front");
    else if (robot_model == ROBOT_MODEL_REAR)
        LOG_INF("Robot Model: Normal/Rear");
    else
        LOG_INF("Not Suport Robot Model, 0x%02x", robot_model);

    struct wrapper_robot_t *wrapper = new_wrapper();

    // wait power ready
    // motor startup maybe trigger pms safety check
    // k_msleep(9000);

    wrap_robot_init(wrapper);
    wrap_drvie_setup(wrapper);


    robot_init(wrapper);

#ifdef CONFIG_APP_ABUP_FOTA		
    abup_check_upgrade_result();
#endif

    while (1)
    {
        led_gpio_toggle_indicator();
        k_msleep(1000);
    }

    return 0;
}
