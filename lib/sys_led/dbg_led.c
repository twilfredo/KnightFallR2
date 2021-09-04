#include <zephyr.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include "dbg_led.h"
#include "sara_r4.h"

LOG_MODULE_REGISTER(DBG_LED, LOG_LEVEL_DBG);
//Extern variable for connection status
//TODO chnage this an event etc...
bool mqttConnected = false;

int init_usr_led(void)
{
    const struct device *dev = device_get_binding(USR_LED);
    return gpio_pin_configure(dev, USR_PIN, GPIO_OUTPUT_ACTIVE | USR_LED_FLAGS);
}

int turn_usr_led_on(void)
{
    const struct device *dev = device_get_binding(USR_LED);
    return gpio_pin_set(dev, USR_PIN, true);
}

int turn_usr_led_off(void)
{
    const struct device *dev = device_get_binding(USR_LED);
    return gpio_pin_set(dev, USR_PIN, false);
}

/**
 * @brief This thread indicates network conenction status.
 * 		Status 1: Fast Blink when not connected.
 * 		Status 2: Slow Blink when connected.
 * 
 */
void thread_flash_debug_led(void *p1, void *p2, void *p3)
{

    const struct device *dev = device_get_binding(GRN_LED);
    const struct device *dev1 = device_get_binding(RED_LED);

    int ret = gpio_pin_configure(dev, GRN_PIN, GPIO_OUTPUT_INACTIVE | GRN_LED_FLAGS);
    int ret1 = gpio_pin_configure(dev1, RED_PIN, GPIO_OUTPUT_INACTIVE | RED_LED_FLAGS);

    if (ret < 0 || ret1 < 0)
    {
        LOG_ERR("LED GPIO ERROR");
        return;
    }

    while (1)
    {

        if (mqttConnected == false)
        {
            gpio_pin_set(dev, RED_PIN, 1);
            k_msleep(100);
            gpio_pin_set(dev, RED_PIN, 0);
        }
        else
        {
            gpio_pin_set(dev, GRN_PIN, 1);
            k_msleep(100);
            gpio_pin_set(dev, GRN_PIN, 0);
        }
        k_msleep(SLEEP_TIME_MS_SLOW);
    }
}