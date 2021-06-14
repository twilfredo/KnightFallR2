/**
 ************************************************************************
 * @file sam_m8q.c
 * @author Wilfred MK
 * @date 14.06.2021 (Last Updated)
 * @brief Custom SAM_M8Q GPS communications module.
 **********************************************************************
 **/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
/* Local Includes */
#include "sam_m8q.h"

LOG_MODULE_REGISTER(SAM_M8Q, LOG_LEVEL_DBG);

/**
 * @brief Initialize uart0 for sam_m8q GPS module communication
 * 
 * @return int errval
 */
int sam_m8q_uart_init(void)
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

/**
 * @brief Sends a command to the GPS Driver module connected on UART0
 * 
 * @param command command to be sent. 
 */
void sam_m8q_uart_tx(char *command)
{
    //printk("Sent: %s\n", command);
    const struct device *uart_dev = device_get_binding(UART0);
    /* Verify uart_poll_out() */
    for (int i = 0; i < strlen(command); i++)
    {
        uart_poll_out(uart_dev, command[i]);
    }
}
