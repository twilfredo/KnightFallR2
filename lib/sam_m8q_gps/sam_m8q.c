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

/* UART RING BUFFERS */
#define RING_BUF_SIZE (64 * 2)
uint8_t rb_buf_sam[RING_BUF_SIZE];
struct ring_buf rx_rb_sam;

/* SAM Send/Rec Semaphore */
K_SEM_DEFINE(samRecSem, 0, 1);

/**
 * @brief Callback function to read data from the modem, and save bytes to allocated
 *          ring buffer. 
 * 
 * @param uart_device 
 * @param user_data 
 */
static void uart_cb(const struct device *uart_device, void *user_data)
{
    ARG_UNUSED(user_data);
    /* Verify uart_irq_update() */
    if (!uart_irq_update(uart_device))
    {
        LOG_ERR("UART IRQ Update ERR");
        return;
    }
    //const struct device *dev_uart1 = device_get_binding(UART1);
    const struct device *dev_uart0 = uart_device;

    int rx, ret;
    static uint8_t read_buf[MAX_READ_SIZE];

    /* get all of the data off UART as fast as we can */
    while (uart_irq_update(dev_uart0) &&
           uart_irq_rx_ready(dev_uart0))
    {
        rx = uart_fifo_read(dev_uart0, read_buf, sizeof(read_buf));

        if (read_buf[0] == '\n')
        {
            //String is newline terminated, a data set is received.
            k_sem_give(&samRecSem);
        }

        if (rx > 0)
        {
            ret = ring_buf_put(&rx_rb_sam, read_buf, rx);

            if (ret != rx)
            {
                LOG_ERR("Rx buffer doesn't have enough space. "
                        "Bytes pending: %d, written: %d",
                        rx, ret);
                sam_mdm_receiver_flush(uart_device);
                //k_sem_give(&modemReadOkSem);
                break;
            }
            //Modem Response received
            //k_sem_give(&modemReadOkSem);
        }
    }
}

/**
 * @brief Initialize uart0 for sam_m8q GPS module communication
 * 
 * @return int errval
 */
int sam_m8q_uart_init(void)
{
    const struct uart_config uart_cfg = {
        .baudrate = 9600,
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
        uart_irq_rx_disable(dev_uart0);
        uart_irq_callback_user_data_set(dev_uart0, uart_cb, NULL);
        uart_irq_rx_enable(dev_uart0);
        ring_buf_init(&rx_rb_sam, 128, rb_buf_sam);
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

/**
 * @brief Drains UART and Discards remaining data.
 * 
 * @param uart_device 
 */
void sam_mdm_receiver_flush(const struct device *uart_device)
{
    uint8_t c;

    //Drain Uart
    while (uart_fifo_read(uart_device, &c, 1) > 0)
    {
        continue;
    }
}

/**
 * @brief Controls and sends data to the SAM-M8Q GPS module. 
 * 
 */
void thread_gps_ctrl(void)
{

    if (sam_m8q_uart_init())
    {
        LOG_ERR("UART Config Error");
    }

    while (1)
    {
        sam_m8q_uart_tx("Welcome\n\r");
        k_sleep(K_SECONDS(1));
    }
}

/**
 * @brief Receives data from the SAM-M8Q GPS Module.
 * 
 */
void thread_gps_receive(void)
{

    while (1)
    {
        k_sem_take(&samRecSem, K_FOREVER);
        sam_recv();
    }
}

/**
 * @brief Receive data from sam gps ring buffer
 * 
 * @return true 
 * @return false 
 */
bool sam_recv(void)
{
    int numReadBytes = 0; //Number of bytes read from ringbuffer
    uint8_t ringBufferLoad[MAX_READ_SIZE];
    char buffer[MAX_READ_SIZE];

    numReadBytes = ring_buf_get(&rx_rb_sam, ringBufferLoad, sizeof(ringBufferLoad));

    for (int i = 0; i < numReadBytes; ++i)
    {
        buffer[i] = ringBufferLoad[i];

        if (i + 1 == numReadBytes)
        {
            buffer[i + 1] = '\0';
        }
    }

    printk("%s", buffer);
    return true;
}