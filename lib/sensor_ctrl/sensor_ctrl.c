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
#include "mcp3008.h"
#include "sam_m8q.h"

LOG_MODULE_REGISTER(sensor_ctrl, LOG_LEVEL_INF);

K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_packet), 10, 4);
//K_MSGQ_DEFINE(turbidity_msgq, sizeof(struct sensor_packet), 10, 4);

K_SEM_DEFINE(sensor_active_sem, 0, 1);
K_SEM_DEFINE(tsd10_read_sem, 0, 1);
K_SEM_DEFINE(gps_read_sem, 0, 1);

/* Creating subcommands (level 1 command) array for command "demo". */
SHELL_STATIC_SUBCMD_SET_CREATE(sensors_sub,
                               SHELL_CMD(single_read, NULL, "Turn sensors on, and attain reading then turn them off",
                                         cmd_sensors_single_read),
                               SHELL_CMD(pwr_on, NULL, "Turn sensors On",
                                         cmd_sensors_on),
                               SHELL_CMD(pwr_off, NULL, "Turn sensors Off",
                                         cmd_sensors_off),
                               SHELL_CMD(gps_pwr_on, NULL, "Power SAM-M8Q On",
                                         cmd_gps_on),
                               SHELL_CMD(gps_pwr_off, NULL, "Power SAM-M8Q On",
                                         cmd_gps_off),
                               SHELL_CMD(tsd_pwr_on, NULL, "Power TSD-10 On",
                                         cmd_tsd_on),
                               SHELL_CMD(tsd_pwr_off, NULL, "Power TSD-10 Off",
                                         cmd_tsd_off),
                               SHELL_SUBCMD_SET_END);

/* Creating root (level 0) command "demo" */
SHELL_CMD_REGISTER(sensor_ctrl, &sensors_sub, "Sensor Control", NULL);

/**
 * @brief Thread that provides abstraction to the sensors, will power control the sensors as required
 *          and put sensor information onto a message queue that can be accesed by the networking thread
 *          where the data can be published using MQTT to thingspeak. 
 */
void thread_sensor_control(void *p1, void *p2, void *p3)
{
    struct sensor_packet sensorData = {0};

    while (1)
    {
        LOG_DBG("Idle");

        //TODO A way to request data from this thread (Will power on sensors and get data, until shutdown received)
        //Thread is active when this sem is attained.
        if (k_sem_take(&sensor_active_sem, K_FOREVER) == 0)
        {
            LOG_DBG("Active");
            /* Read Request Received */
            //1. Power on GPS
            if (get_gps_pwr_stat() == false)
                sam_m8q_pwr_on();

            //Power Stability delay
            k_msleep(VRAIL_DELAY);

            /* GPS Should be called before getTurbidity, as the gps lock can take an arbitrary amount of time */

            //2. Collect GPS
            get_gps(&sensorData);

            //3. Power off GPS
            sam_m8q_pwr_off();

            //4. Power on TSD-10
            tsd_10_pwr_on();

            //Power Stability delay
            k_msleep(VRAIL_DELAY);

            /* This call waits on a singal */
            //5. Collect Turbidity

            get_turbidity(&sensorData);
            //6. Turn TSD-10 Off
            tsd_10_pwr_off();

            //7. Post gathered Data
            if (k_msgq_put(&sensor_msgq, &sensorData, K_NO_WAIT) != 0)
            {
                k_msgq_purge(&sensor_msgq); //Make Space
                //TODO Attempt to put it here again?
            }
            //Clear current data, queue is pass by copy not reference
            memset(&sensorData, 0, sizeof(struct sensor_packet));
        }
    }
}

/**
 * @brief Get the long and latt values from the sam-m8q driver
 * @note Function waits on a message from the gps driver for a timeout period. 
 * 
 * TODO Complete this/Add reasonable timeout.
 * 
 * @param sensorData sensor packet for the collected data to be stored into
 */
void get_gps(struct sensor_packet *sensorData)
{
    k_sem_give(&gps_read_sem);

    struct samGLLMessage gllMsgPacket = {0};
    /* Wait for receive data from GPS thread */
    LOG_INF("Waiting for GPS Lock");
    if (k_msgq_get(&gps_msgq, &gllMsgPacket, K_SECONDS(GPS_NO_LOCK_TIMEOUT)) != 0)
    {
        /* MSG not received, timeout */
        LOG_ERR("GPS No lock timeout");
        sensorData->lattitude = GPS_NO_LOCK_VAL;
        sensorData->longitude = GPS_NO_LOCK_VAL;
        /* Notify the GPS polling thread to stop polling */
        gpsTimeOutOccured = true;
        return;
    }

    //printk("REC GPS LOCK: %f  |--| %f", gllMsgPacket.lat, gllMsgPacket.lon);
    sensorData->lattitude = gllMsgPacket.lat;
    sensorData->longitude = gllMsgPacket.lon;
}

/**
 * @brief Get the Turbidity reading from the tsd-10 driver, using a signal event
 *          as only one value is read. Reduced stack usage as opposed to 
 *          message queue/FIFO
 * 
 * @param sensorData sensor packet for the collected data to be stored into
 */
void get_turbidity(struct sensor_packet *sensorData)
{
    //This sem allows for a tsd10 adc read, see tsd10_adc.c
    struct tsd_data tsdDataRx = {0};

    k_sem_give(&tsd10_read_sem);

    if (k_msgq_get(&tsd_msgq, &tsdDataRx, K_SECONDS(TSD_EVENT_TIMEOUT)) != 0)
    {
        /* MSG not received, timeout */
        LOG_ERR("TSD_10 Reading timed out");
        return;
    }

    //Save data into respective field in sensor data packet
    sensorData->tsdmV = tsdDataRx.tsd_mV;
    sensorData->turbidity = tsdDataRx.tsd_NTU;

    //printk("NTUs: %d Voltage: %f mV\n", sensorData->turbidity, sensorData->tsdmV);
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

/**
 * @brief Manually power on tsd10
 * 
 * @param shell unused
 * @param argc unused
 * @param argv unused
 * @param data unused
 * @return int 0 on completion
 */
int cmd_tsd_on(const struct shell *shell,
               size_t argc, char **argv, void *data)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(shell);
    ARG_UNUSED(data);

    tsd_10_pwr_on();
    return 0;
}

/**
 * @brief Manually power off tsd10
 * 
 * @param shell unused
 * @param argc unused
 * @param argv unused
 * @param data unused
 * @return int 0 on completion
 */
int cmd_tsd_off(const struct shell *shell,
                size_t argc, char **argv, void *data)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(shell);
    ARG_UNUSED(data);

    tsd_10_pwr_off();
    return 0;
}

/**
 * @brief Manually power on SAM-M8Q GPS
 * 
 * @param shell unused
 * @param argc unused
 * @param argv unused
 * @param data unused
 * @return int 0 on completion
 */
int cmd_gps_on(const struct shell *shell,
               size_t argc, char **argv, void *data)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(shell);
    ARG_UNUSED(data);

    sam_m8q_pwr_on();
    return 0;
}

/**
 * @brief Manually power off SAM-M8Q GPS
 * 
 * @param shell unused
 * @param argc unused
 * @param argv unused
 * @param data unused
 * @return int 0 on completion
 */
int cmd_gps_off(const struct shell *shell,
                size_t argc, char **argv, void *data)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ARG_UNUSED(shell);
    ARG_UNUSED(data);

    sam_m8q_pwr_off();
    return 0;
}
