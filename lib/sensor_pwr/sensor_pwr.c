/**
 ************************************************************************
 * @file sensor_pwr.c
 * @author Wilfred MK
 * @date 23.06.2021 (Last Updated)
 * @brief Driver controls the GPIO pins that drives Vgs to the mosfets that
 *          power control the sensors (GPS, Turbidity)
 **********************************************************************
 **/
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

//Local Includes
#include "sensor_pwr.h"

/**
 * @brief Initialises the GPIO pins used to control the mosfets associated with
 *          power controlling the sensors
 * 
 * @return 0 on success, < 0 else
 */
int init_sensor_pwr_gpio(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    int err = gpio_pin_configure(gpio, SAM_M8Q_PWR_PIN, GPIO_OUTPUT_INACTIVE);
    int err1 = gpio_pin_configure(gpio, TSD_10_PWR_PIN, GPIO_OUTPUT_INACTIVE);

    if (err < 0 || err1 < 0)
    {
        //Config Error
        return -1;
    }

    return 0;
}

/**
 * @brief Turns on power to the SAM_M8Q GPS Module
 * 
 * @return 0 on succes
 */
int sam_m8q_pwr_on(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    return gpio_pin_set(gpio, SAM_M8Q_PWR_PIN, PWR_ON);
}

/**
 * @brief Turns power on to TSD-10 Turbidity sensor
 * 
 * @return 0 on succes
 */
int tsd_10_pwr_on(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    return gpio_pin_set(gpio, TSD_10_PWR_PIN, PWR_ON);
}

/**
 * @brief Turns off power to the SAM_M8Q GPS Module
 * 
 * @return 0 on succes
 */
int sam_m8q_pwr_off(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    return gpio_pin_set(gpio, SAM_M8Q_PWR_PIN, PWR_OFF);
}

/**
 * @brief Turns power off to TSD-10 Turbidity sensor
 * 
 * @return 0 on succes
 */
int tsd_10_pwr_off(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    return gpio_pin_set(gpio, TSD_10_PWR_PIN, PWR_OFF);
}