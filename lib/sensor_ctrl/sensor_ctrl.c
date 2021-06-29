/**
 ************************************************************************
 * @file sensor_ctrl.c
 * @author Wilfred MK
 * @date 23.06.2021 (Last Updated)
 * @brief Abstraction driver for the sensors attached, will power control 
 *          communicate with the sensors and produce data that can be 
 *          published using the MQTT (Turbidity, GPS Coords)
 **********************************************************************
 **/
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <string.h>
#include <logging/log.h>
#include <shell/shell.h>
//Local Includes
#include "sensor_ctrl.h"
#include "sensor_pwr.h"

LOG_MODULE_REGISTER(sensor_ctrl, LOG_LEVEL_DBG);

K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_packet), 10, 4);
K_SEM_DEFINE(sensor_active_sem, 0, 1);

/* Creating subcommands (level 1 command) array for command "demo". */
SHELL_STATIC_SUBCMD_SET_CREATE(sensors_sub,
                               SHELL_CMD(single_read, NULL, "Turn sensors on, and attain reading then turn them off",
                                         cmd_sensors_single_read),
                               SHELL_CMD(pwr_on, NULL, "Turn sensors on",
                                         cmd_sensors_on),
                               SHELL_CMD(pwr_off, NULL, "Turn sensors off",
                                         cmd_sensors_off),
                               SHELL_SUBCMD_SET_END);

/* Creating root (level 0) command "demo" */
SHELL_CMD_REGISTER(sensor_ctrl, &sensors_sub, "Sensor Control", NULL);

/**
 * @brief Thread that provides abstraction to the sensors, will power control the sensors as required
 *          and put sensor information onto a message queue that can be accesed by the networking thread
 *          where the data can be published using MQTT to thingspeak. 
 */
void thread_sensor_control(void)
{
    //init_sensor_pwr_gpio();
    struct sensor_packet sensorData = {0};

    while (1)
    {
        LOG_DBG("Idle");

        //TODO A way to request data from this thread (Will power on sensors and get data, until shutdown received)
        sensorData.turbidity++;

        if (k_sem_take(&sensor_active_sem, K_FOREVER) == 0)
        {
            LOG_DBG("Active");
            //Read Request Received
            if (turn_sensors_on())
            {
                LOG_ERR("Unable to power on sensors");
            }

            //TODO Delay to compensate for ADC Settling and GPS Lock?
            k_msleep(500);

            //TODO getTurbidity(&sensorData), getGPS(&sensorData)

            if (k_msgq_put(&sensor_msgq, &sensorData, K_NO_WAIT) != 0)
            {
                k_msgq_purge(&sensor_msgq); //Make Space
            }

            //Clear current data, queue is pass by copy not reference
            memset(&sensorData, 0, sizeof sensorData);

            if (turn_sensors_off())
            {
                LOG_ERR("Unable to power off sensors");
            }
        }
    }
}

/**
 * @brief Command to process a manual sensor read through shell,
 *          releasing sensor_active_sem (semaphore) causes the
 *          sensor_ctrl thread to perform a single read after turning
 *          sensors on. 
 * 
 * @param shell unused
 * @param argc unused
 * @param argv unused
 * @param data unused
 * @return int 0 on completion
 */
int cmd_sensors_single_read(const struct shell *shell,
                            size_t argc, char **argv, void *data)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(shell);
    ARG_UNUSED(data);

    /* Give indicator semaphore to sensor_ctrl thread */
    k_sem_give(&sensor_active_sem);
    return 0;
}

/**
 * @brief Manually power on sensors
 * 
 * @param shell unused
 * @param argc unused
 * @param argv unused
 * @param data unused
 * @return int 0 on completion
 */
int cmd_sensors_off(const struct shell *shell,
                    size_t argc, char **argv, void *data)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(shell);
    ARG_UNUSED(data);

    turn_sensors_off();
    return 0;
}

/**
 * @brief Manually power off sensors
 * 
 * @param shell unused
 * @param argc unused
 * @param argv unused
 * @param data unused
 * @return int 0 on completion
 */
int cmd_sensors_on(const struct shell *shell,
                   size_t argc, char **argv, void *data)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(shell);
    ARG_UNUSED(data);

    turn_sensors_on();
    return 0;
}
