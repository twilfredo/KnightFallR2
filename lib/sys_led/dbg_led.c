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

    bool led_is_on = true;

    const struct device *dev = device_get_binding(BLUE_LED);
    const struct device *dev1 = device_get_binding(RED_LED);

    int ret = gpio_pin_configure(dev, BLUE_PIN, GPIO_OUTPUT_INACTIVE | BLUE_LED_FLAGS);
    int ret1 = gpio_pin_configure(dev1, RED_PIN, GPIO_OUTPUT_INACTIVE | RED_LED_FLAGS);

    if (ret < 0 || ret1 < 0)
    {
        LOG_ERR("LED GPIO ERROR");
        return;
    }

    while (1)
    {
        led_is_on = !led_is_on;

        if (!mqttConnected)
        {
            gpio_pin_set(dev, RED_PIN, (int)led_is_on);

            k_msleep(SLEEP_TIME_MS_FAST);
        }
        else
        {
            gpio_pin_set(dev, BLUE_PIN, (int)led_is_on);
            gpio_pin_set(dev, RED_PIN, (int)led_is_on);
            k_msleep(SLEEP_TIME_MS_SLOW);
        }
    }
}