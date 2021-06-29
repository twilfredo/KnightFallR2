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
#include <logging/log.h>

//Local Includes
#include "sensor_pwr.h"

LOG_MODULE_REGISTER(sensor_pwr, LOG_LEVEL_DBG);

/**
 * @brief Turne all sensors on
 * 
 * @return 0 on success
 */
int turn_sensors_on(void)
{
    LOG_DBG("Sensors On");
    int err1 = tsd_10_pwr_on();
    int err = sam_m8q_pwr_on();
    //Zero on succes, both must be zero
    return err || err1;
}

/**
 * @brief Turn all sensors off
 * 
 * @return 0 on success 
 */
int turn_sensors_off(void)
{
    LOG_DBG("Sensors Off");
    int err1 = tsd_10_pwr_off();
    int err = sam_m8q_pwr_off();
    //Zero on succes, both must be zero
    return err || err1;
}

/**
 * @brief Initialises the GPIO pins used to control the mosfets associated with
 *          power controlling the sensors
 * 
 * @note function deprecated,
 *   gpio config init and deinit built into respective functions
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
    int err = gpio_pin_configure(gpio, SAM_M8Q_PWR_PIN, GPIO_OUTPUT_INACTIVE);
    int err1 = gpio_pin_set(gpio, SAM_M8Q_PWR_PIN, PWR_ON);

    if (err1 != 0 || err != 0)
    {
        LOG_ERR("Error configuring GPIO SAM_M8Q");
        return -1;
    }

    return 0;
}

/**
 * @brief Turns power on to TSD-10 Turbidity sensor
 * 
 * @return 0 on succes
 */
int tsd_10_pwr_on(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    int err = gpio_pin_configure(gpio, TSD_10_PWR_PIN, GPIO_OUTPUT_INACTIVE);
    int err1 = gpio_pin_set(gpio, TSD_10_PWR_PIN, PWR_ON);

    if (err1 != 0 || err != 0)
    {
        LOG_ERR("Error configuring GPIO TSD_10");
        return -1;
    }

    return 0;
}

/**
 * @brief Turns off power to the SAM_M8Q GPS Module
 * 
 * @return 0 on succes
 */
int sam_m8q_pwr_off(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    int err = gpio_pin_set(gpio, SAM_M8Q_PWR_PIN, PWR_OFF);
    int err1 = gpio_pin_configure(gpio, SAM_M8Q_PWR_PIN, GPIO_DISCONNECTED);

    if (err1 != 0 || err != 0)
    {
        LOG_ERR("Error configuring GPIO SAM_M8Q");
        return -1;
    }

    return 0;
}

/**
 * @brief Turns power off to TSD-10 Turbidity sensor
 * 
 * @return 0 on succes
 */
int tsd_10_pwr_off(void)
{
    const struct device *gpio = device_get_binding(GPIO_FET);
    int err = gpio_pin_set(gpio, TSD_10_PWR_PIN, PWR_OFF);
    int err1 = gpio_pin_configure(gpio, TSD_10_PWR_PIN, GPIO_DISCONNECTED);

    if (err1 != 0 || err != 0)
    {
        LOG_ERR("Error configuring GPIO TSD_10");
        return -1;
    }

    return 0;
}