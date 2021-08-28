
/**
 ************************************************************************
 * @file sara_r4.c
 * @author Wilfred MK
 * @date 13.05.2021 (Last Updated)
 * @brief Entry thread, instantiates auxilliary modules.
 **********************************************************************
 **/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

#include "dbg_led.h"
#include "sara_r4.h"
#include "sam_m8q.h"
//Todo Remove Later, main thread does not need to communicate with sensors.
#include "sys_pwr.h"
#include "sensor_ctrl.h"
#include "sensor_pwr.h"
#include "mcp3008.h"

#define SENSOR_ACTIVE_DELAY 13000

LOG_MODULE_REGISTER(device_ctrl_main, LOG_LEVEL_DBG);

K_MSGQ_DEFINE(to_network_msgq, sizeof(struct sensor_packet), 10, 4);

/* Aux Threads */
K_THREAD_STACK_DEFINE(led_thread_stack, STACK_SIZE_LED_THREAD);
struct k_thread led_thread_d;

/* Network Threads - Modem */
K_THREAD_STACK_DEFINE(modem_ctrl_stack, STACK_SIZE_MODEM_THREAD);
K_THREAD_STACK_DEFINE(modem_recv_stack, STACK_SIZE_MODEM_THREAD);
struct k_thread modem_ctrl_d, modem_recv_d;

/* Sensor Control Thread */
K_THREAD_STACK_DEFINE(sensor_ctrl_stack, STACK_SIZE_SENSOR_CTRL);
struct k_thread sensor_ctrl_d;

/* GPS Communications Thread */
K_THREAD_STACK_DEFINE(gps_ctrl_stack, STACK_SIZE_GPS_THREAD);
struct k_thread gps_ctrl_d;

/* MCP3008 ADC */
K_THREAD_STACK_DEFINE(adc_ctrl_stack, STACK_SIZE_GPS_THREAD);
struct k_thread adc_ctrl_d;
/* Thread IDS */
k_tid_t led_tid, modem_ctrl_tid, modem_recv_tid, sensor_ctrl_tid, gps_ctrl_tid, adc_ctrl_tid;

/**
 * @brief Creates system threads.
 * 
 */
void spawn_threads(void)
{
    /* Spawn System threads */
    led_tid = k_thread_create(&led_thread_d, led_thread_stack, K_THREAD_STACK_SIZEOF(led_thread_stack),
                              thread_flash_debug_led,
                              NULL, NULL, NULL,
                              THREAD_PRIORITY_LED_THREAD, 0, K_NO_WAIT);

    modem_ctrl_tid = k_thread_create(&modem_ctrl_d, modem_ctrl_stack, K_THREAD_STACK_SIZEOF(modem_ctrl_stack),
                                     thread_modem_ctrl,
                                     NULL, NULL, NULL,
                                     THREAD_PRIORITY_MODEM, 0, K_NO_WAIT);

    modem_recv_tid = k_thread_create(&modem_recv_d, modem_recv_stack, K_THREAD_STACK_SIZEOF(modem_recv_stack),
                                     thread_modem_receive,
                                     NULL, NULL, NULL,
                                     THREAD_PRIORITY_MODEM, 0, K_NO_WAIT);

    sensor_ctrl_tid = k_thread_create(&sensor_ctrl_d, sensor_ctrl_stack, K_THREAD_STACK_SIZEOF(sensor_ctrl_stack),
                                      thread_sensor_control,
                                      NULL, NULL, NULL,
                                      PRIORITY_SENSOR_CTRL, 0, K_NO_WAIT);

    gps_ctrl_tid = k_thread_create(&gps_ctrl_d, gps_ctrl_stack, K_THREAD_STACK_SIZEOF(gps_ctrl_stack),
                                   thread_gps_ctrl,
                                   NULL, NULL, NULL,
                                   THREAD_PRIORITY_GPS, 0, K_NO_WAIT);

    adc_ctrl_tid = k_thread_create(&adc_ctrl_d, adc_ctrl_stack, K_THREAD_STACK_SIZEOF(adc_ctrl_stack),
                                   thread_adc_ctrl,
                                   NULL, NULL, NULL,
                                   THREAD_PRIORITY_ADC, 0, K_NO_WAIT);
}

/**
 * @brief Entry thread to start the USB driver which the Shell 
 *          instance is dependant on. 
 * 
 *        Main thread to control the sensors and the networking,
 */
void main(void)
{
    int sysPwrErr;
pmic_pwr_setup:

    sysPwrErr = sys_pwr_init();

    if (sysPwrErr != OK)
    {
        k_sleep(K_SECONDS(1));
        goto pmic_pwr_setup;
    }

    /* Start USB Driver */
    //TODO: Disable for production: and when using battery pack
    usb_enable(NULL);

    /* Start Threads */
    spawn_threads();

    struct sensor_packet sensorDataRec = {0};

    /* Wait for network to init, sem given by the sara_r4.c driver */
    LOG_INF("Waiting for network...");
    k_sem_take(&networkReady, K_FOREVER);
    LOG_INF("Network Ready, System Initialised");

    while (1)
    {
        //TODO Add Sequence Control, Primary Loop (Network, Read, Sleep)
        /* 1. Get Sensor Reading */
        k_sem_give(&sensor_active_sem);
        k_msgq_get(&sensor_msgq, &sensorDataRec, K_FOREVER);

        printk("Sensors: Turbidity %d NTUs, Lon: %f Lat: %f", sensorDataRec.turbidity, sensorDataRec.longitude, sensorDataRec.lattitude);

        /* 2. Send Data to Network Driver */
        if (k_msgq_put(&to_network_msgq, &sensorDataRec, K_NO_WAIT) != 0)
        {
            k_msgq_purge(&to_network_msgq); //Make Space
        }

        /* 3. Sleep Device */
        //TODO Sleep Sara R4, Sleep GPS (Should do this in the sensor driver after reading is done)

        //Clear current data, queue is pass by copy not reference
        memset(&sensorDataRec, 0, sizeof sensorDataRec);
        k_msleep(SENSOR_ACTIVE_DELAY);
    }
}
