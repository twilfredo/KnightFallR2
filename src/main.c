
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
#include <usb/usb_device.h>
#include <drivers/uart.h>
#include <logging/log.h>

/* Local Includes */
#include "dbg_led.h"
#include "sara_r4.h"
#include "sensors_custom.h"

LOG_MODULE_REGISTER(UART_TEST, LOG_LEVEL_DBG);

/* Compile Time Thread Init */
K_THREAD_DEFINE(debug_led, STACK_SIZE_LED_THREAD, thread_flash_debug_led, NULL, NULL, NULL, THREAD_PRIORITY_LED_THREAD, 0, 50);
//K_THREAD_DEFINE(sensor_driver, STACK_SIZE_SENSORS, thread_sensors, NULL, NULL, NULL, THREAD_PRIORITY_SENSORS, 0, 10);

/* Compile Time Thread Init */
//K_THREAD_DEFINE(modem_send, STACK_SIZE_MODEM_THREAD, thread_modem_ctrl, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 50);
//K_THREAD_DEFINE(modem_receive, STACK_SIZE_MODEM_THREAD, thread_modem_receive, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 200);
/* Device tree node identifier for UART1 */
#define UART0 DT_LABEL(DT_NODELABEL(uart0))
/**
 * @brief Initialize uart1 for ublox sara communication
 * 
 * @return int errval
 */
int uart_init(void)
{
    const struct uart_config uart_cfg = {
        .baudrate = 115200,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

    /* Obtain pre-initialised device binding - see device_config.h */
    const struct device *dev_uart0 = device_get_binding(UART0);
    if (!dev_uart0)
    {
        /*Unable to retrieve device structure */
        return -ENODEV;
    }
    else
    {
        return uart_configure(dev_uart0, &uart_cfg);
    }
}

void test_uart_tx(char *command)
{
    //printk("Sent: %s\n", command);
    const struct device *uart_dev = device_get_binding(UART0);
    /* Verify uart_poll_out() */
    for (int i = 0; i < strlen(command); i++)
    {
        uart_poll_out(uart_dev, command[i]);
    }
}

void main(void)
{
    /* Start USB Driver */
    usb_enable(NULL);
    uart_init();

    for (;;)
    {
        test_uart_tx("Hello\n");
        k_sleep(K_SECONDS(1));
    }
}
