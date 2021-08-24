
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
//#include "sensors_custom.h"
#include "sam_m8q.h"
//#include "tsd10_adc.h"

//Todo Remove Later, main thread does not need to communicate with sensors.
#include "sensor_ctrl.h"
#include "sensor_pwr.h"
#include "mcp3008.h"

#define SENSOR_ACTIVE_DELAY 13000

LOG_MODULE_REGISTER(device_ctrl_main, LOG_LEVEL_DBG);

K_MSGQ_DEFINE(to_network_msgq, sizeof(struct sensor_packet), 10, 4);

/* Compile Time Threads - These threads start runtime after the delay specified, else at ~t=0 */
/* Aux Threads */
K_THREAD_DEFINE(debug_led, STACK_SIZE_LED_THREAD, thread_flash_debug_led, NULL, NULL, NULL, THREAD_PRIORITY_LED_THREAD, 0, 50);

/* Network Threads - Modem */
K_THREAD_DEFINE(modem_ctrl, STACK_SIZE_MODEM_THREAD, thread_modem_ctrl, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 50);
K_THREAD_DEFINE(modem_receive, STACK_SIZE_MODEM_THREAD, thread_modem_receive, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 200);

/* TSD-10 ADC Thread */
//! Enable CMAKE COMPILE FOR THIS FILE WHEN TESTING
K_THREAD_DEFINE(sensor_ctrl, STACK_SIZE_SENSOR_CTRL, thread_sensor_control, NULL, NULL, NULL, PRIORITY_SENSOR_CTRL, 0, 50);

/* GPS Communications Thread */
K_THREAD_DEFINE(gps_ctrl, STACK_SIZE_GPS_THREAD, thread_gps_ctrl, NULL, NULL, NULL, THREAD_PRIORITY_GPS, 0, 50);

/*MCP3008 ADC */
K_THREAD_DEFINE(adc_ctrl, STACK_SIZE_ADC_THREAD, thread_adc_ctrl, NULL, NULL, NULL, THREAD_PRIORITY_ADC, 0, 50);

/**
 * @brief Entry thread to start the USB driver which the Shell 
 *          instance is dependant on. 
 * 
 *        Main thread to control the sensors and the networking,
 */
void main(void)
{
    /* Start USB Driver */
    //TODO: Disable for production: and when using battery pack
    usb_enable(NULL);

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
