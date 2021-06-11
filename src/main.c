
/**
 ************************************************************************
 * @file sara_r4.c
 * @author Wilfred MK
 * @date 13.05.2021 (Last Updated)
 * @brief Entry thread, instantiates auxilliary modules.
 * TODO: Start aux threads in this file
 **********************************************************************
 **/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include "dbg_led.h"
#include "sara_r4.h"
#include "sensors_custom.h"

/* Compile Time Thread Init */
K_THREAD_DEFINE(debug_led, STACK_SIZE_LED_THREAD, thread_flash_debug_led, NULL, NULL, NULL, THREAD_PRIORITY_LED_THREAD, 0, 50);
K_THREAD_DEFINE(sensor_driver, STACK_SIZE_SENSORS, thread_sensors, NULL, NULL, NULL, THREAD_PRIORITY_SENSORS, 0, 10);

/* Compile Time Thread Init */
K_THREAD_DEFINE(modem_send, STACK_SIZE_MODEM_THREAD, thread_modem_ctrl, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 50);
K_THREAD_DEFINE(modem_receive, STACK_SIZE_MODEM_THREAD, thread_modem_receive, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 200);
