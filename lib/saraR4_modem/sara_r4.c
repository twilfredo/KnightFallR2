
#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/ring_buffer.h>
#include "sara_r4.h"
#include "sensors_custom.h"

//UART RING BUFFERS
#define RING_BUF_SIZE (64 * 2)
uint8_t rb_buf[RING_BUF_SIZE];
struct ring_buf rx_rb;

LOG_MODULE_REGISTER(SARA_R4, LOG_LEVEL_DBG);

/* Compile Time Thread Init */
K_THREAD_DEFINE(modem_send, STACK_SIZE_MODEM_THREAD, thread_modem_send, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 50);
K_THREAD_DEFINE(modem_receive, STACK_SIZE_MODEM_THREAD, thread_modem_receive, NULL, NULL, NULL, THREAD_PRIORITY_MODEM, 0, 200);

/* Modem Send/Rec Semaphore */
K_SEM_DEFINE(modemSendSem, 1, 1);
K_SEM_DEFINE(modemRecSem, 0, 1);
K_SEM_DEFINE(modemReadOkSem, 0, 1);
K_SEM_DEFINE(modemCommandOkSem, 0, 1);

#define MODEM_APN "telstra.internet"
#define MODEM_MCCMNO "50501"
#define HOME_PC_IP "167.179.184.183"
#define AT_INIT_CMD_SIZE 9
#define TCP_INI_CMD_SIZE 2

//TODO Increase the TCP Timeout
#define TCP_TIMEOUT_S 10
#define NETWORK_TIMEOUT_S 60

/**
 * @brief The following array contains modem initialization AT commands.
 *        1. Turn Echo Off.
 *        2. Stop Functionality.
 *        3. Extended Error Numbers.
 *        4. Use External Sim (Required by Boron).
 *        5. UNC Messages for registration. 
 *        6. Set Custom APN
 *        7. Start Functionality
 *        8. Set MCCMNO
 *        9. Check Network Registration Status.
 *        10. RSSI (Current Service Cell)
 */
char atInitCommands[AT_INIT_CMD_SIZE][64] = {
    "ATE0\r",             //1
    "AT+CFUN=0\r",        //2
    "AT+CMEE=1\r",        //3
    "AT+UGPIOC=23,0,0\r", //4
    "AT+CREG=1\r",        //5
    //"AT+CGMI\r",                               //Request Manufacture Data
    //"AT+CGMM\r",                               //Request Manufacture Data
    //"AT+CGMR\r",                               //Request Manufacture Data
    //"AT+CGSN\r",                               //Request Manufacture Data
    //"AT+CIMI\r",                               //Request Manufacture Data
    "AT+CGDCONT=1,\"IP\",\"" MODEM_APN "\"\r", //6
    "AT+CFUN=1\r",                             //7
    "AT+COPS=1,2,\"" MODEM_MCCMNO "\"\r",
    "AT+CFUN=0\r"};

char tcpSetupCommands[TCP_INI_CMD_SIZE][128] = {
    "AT+USOCR=6\r",
    "AT+USOCO=0,\"" HOME_PC_IP "\",4011\r"};

/**
 * @brief Primary thread that communicates withe Sara-R4 Modem
 * 
 */
void thread_modem_send(void)
{

restart_modem:
    tcpConnected = false;
    //Config Modem GPIO Pins
    if (modem_config_pins() != 0)
    {
        LOG_ERR("Modem GPIO Config Error");
    }

    //Power Cycle Sara R4
    modem_pin_init();

    //Enable UART to Modem
    if (modem_uart_init() != 0)
    {
        LOG_ERR("UART Config Error");
    }

    //Delay For Modem Response Time After Init.
    k_msleep(4000);
    //Modem is powered on here

    if (!modem_network_init())
    {
        LOG_WRN("Error Initializing Network, Restart Modem");
        //Unable Modem R/W
        k_sem_give(&modemSendSem);
        k_sem_give(&modemRecSem);
        goto restart_modem;
    }

    short connectionAttempts = 0;

reconnect_tcp:
    if (!modem_tcp_init())
    {
        LOG_WRN("Error Connecting to TCP Server, Trying Again");
        //Unable Modem R/W
        k_sem_give(&modemSendSem);
        k_sem_give(&modemRecSem);
        connectionAttempts++;

        if (connectionAttempts >= 10)
        {
            LOG_ERR("Too Many Connection Attempts, Restarting Modem");
            goto restart_modem;
        }
        else
        {
            goto reconnect_tcp;
        }
    }

    tcpConnected = true;

    struct sensor_data sensorData = {0};
    char sendBuffer[128];
    char tempBuffer[64];

    while (1)
    {

        /* Receive Data from sensor message queue */
        if (k_sem_take(&modemSendSem, K_SECONDS(TCP_TIMEOUT_S)) == 0)
        {

            if (k_msgq_get(&sensor_msgq, &sensorData, K_SECONDS(5)) == 0)
            {
                /*Convert sensor data to string format*/
                snprintk(tempBuffer, 40, "[Temp:%.2fC Roll:%.2f Pitch:%.2f]", sensorData.ambientTemperature, sensorData.roll, sensorData.pitch);
                //AT+USOWR=0,X : where X is length of data.
                snprintk(sendBuffer, 64, "AT+USOWR=0,%d,\"%s\"\r", strlen(tempBuffer), tempBuffer);
            }

            modem_uart_tx(sendBuffer);
            k_sem_give(&modemRecSem);
            k_msleep(500);
        }
        else
        {
            //TCP Error, attempt to re-establish connection.
            tcpConnected = false;
            goto reconnect_tcp;
        }

        //TODO and print RSSI Value.
        //modem_uart_tx("AT+CESQ\r");
    }
}

/**
 * @brief Receive data from the modem on UART RX
 * 
 */
void thread_modem_receive(void)
{

    while (1)
    {

        //Wait until a command is sent/
        if (k_sem_take(&modemRecSem, K_FOREVER) == 0)
        {
            //Wait until Rx Data is fully received.
            if (k_sem_take(&modemReadOkSem, K_FOREVER) == 0)
            {
                //#Wait for RX to complete reading.
                k_msleep(1000);
                //Read RX ISR Ring Buffer.
                modem_recv();
                //Next command can be sent now.
            }
        }
    }
}

/**
 * @brief Initialises network registration, waits on semaphore to detect
 *          if the modem was ok with the command send. Returns false else, 
 *          and the calling function will jump to a previous 'goto' to attempt
 *          to redo the following.
 * 
 * @return true Registration Complete
 * @return false Registration Failed
 */
bool modem_network_init(void)
{
    for (int i = 0; i < AT_INIT_CMD_SIZE; ++i)
    {
        //Run Modem Initialization Sequence
        if (k_sem_take(&modemSendSem, K_SECONDS(NETWORK_TIMEOUT_S)) == 0)
        {
            //Modem has received data, send next command
            modem_uart_tx(atInitCommands[i]);
            k_sem_give(&modemRecSem);

            if (k_sem_take(&modemCommandOkSem, K_SECONDS(NETWORK_TIMEOUT_S)) != 0)
            {
                //Modem response was not OK;
                return false;
            }
        }
        else
        {
            //Unable to send command
            return false;
        }
    }
    //All commands returned OK and the modemSendSem  was released
    return true;
}

/**
 * @brief Attempts to connect to the TCP address defined in the tcpSetupCommands array.
 *          Returns true if successfully connected, else the caller function will
 *          just to a 'goto' to restablish tcp connection. 
 * 
 * @return true Connection success.
 * @return false Connection failed.
 */
bool modem_tcp_init(void)
{
    for (int i = 0; i < TCP_INI_CMD_SIZE; ++i)
    {

        if (k_sem_take(&modemSendSem, K_SECONDS(TCP_TIMEOUT_S)) == 0)
        {
            modem_uart_tx(tcpSetupCommands[i]);
            k_sem_give(&modemRecSem);

            if (k_sem_take(&modemCommandOkSem, K_SECONDS(TCP_TIMEOUT_S)) != 0)
            {
                //Modem response was not OK;
                return false;
            }
        }
        else
        {
            //Unable to send command
            return false;
        }
    }
    //All commands returned OK and the modemSendSem  was released
    return true;
}

/**
 * @brief Read modem response saved into ring buffer.
 * 
 */
void modem_recv(void)
{
    int readBytes = 0;
    uint8_t ringBufferLoad[MAX_READ_SIZE];
    char buffer[MAX_READ_SIZE];

    readBytes = ring_buf_get(&rx_rb, ringBufferLoad, sizeof(ringBufferLoad));

    //Null Terminate Read Buffer
    //TODO Figure out a way to build a string from this.
    for (int i = 0; i < readBytes; ++i)
    {
        buffer[i] = ringBufferLoad[i];

        if (i + 1 == readBytes)
        {
            buffer[i + 1] = '\0';
        }
    }

    for (int i = 0; i < strlen(buffer); ++i)
    {
        if (buffer[i] == 'O' && buffer[i + 1] == 'K')
        {
            k_sem_give(&modemSendSem);
            k_sem_give(&modemCommandOkSem);
        }
    }

    printk("-----Modem Response Start------\n");
    printk("%s", buffer);
    printk("-----Modem Response End------\n");
}

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
    const struct device *dev_uart1 = uart_device;

    int rx, ret;
    static uint8_t read_buf[MAX_READ_SIZE];

    /* get all of the data off UART as fast as we can */
    while (uart_irq_update(dev_uart1) &&
           uart_irq_rx_ready(dev_uart1))
    {
        rx = uart_fifo_read(dev_uart1, read_buf, sizeof(read_buf));
        //Todo Modem Replies, Need to fix ring buffer.

        if (rx > 0)
        {
            ret = ring_buf_put(&rx_rb, read_buf, rx);

            if (ret != rx)
            {
                LOG_ERR("Rx buffer doesn't have enough space. "
                        "Bytes pending: %d, written: %d",
                        rx, ret);
                mdm_receiver_flush(uart_device);
                k_sem_give(&modemReadOkSem);
                break;
            }
            k_sem_give(&modemReadOkSem);
        }
    }
}

/**
 * @brief Drains UART and Discards remaining data.
 * 
 * @param uart_device 
 */
void mdm_receiver_flush(const struct device *uart_device)
{
    uint8_t c;

    //Drain Uart
    while (uart_fifo_read(uart_device, &c, 1) > 0)
    {
        continue;
    }
}

/**
 * @brief Initialize uart1 for ublox sara communication
 * 
 * @return int errval
 */
int modem_uart_init(void)
{
    const struct uart_config uart_cfg = {
        .baudrate = 115200,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS};

    /* Obtain pre-initialised device binding - see device_config.h */
    const struct device *dev_uart1 = device_get_binding(UART1);
    if (!dev_uart1)
    {
        /*Unable to retrieve device structure */
        return -ENODEV;
    }
    else
    {
        uart_irq_rx_disable(dev_uart1);
        uart_irq_callback_user_data_set(dev_uart1, uart_cb, NULL);
        uart_irq_rx_enable(dev_uart1);
        ring_buf_init(&rx_rb, 128, rb_buf);
        return uart_configure(dev_uart1, &uart_cfg);
    }
}

/**
 * @brief Configure the power control pins for the modems.
 * 
 * @return int Errval
 */
int modem_config_pins(void)
{
    const struct device *gpio_dev;

    /* Enable the serial buffer for SARA-R4 modem */
    gpio_dev = device_get_binding(SERIAL_BUFFER_ENABLE_GPIO_NAME);

    if (!gpio_dev)
    {
        return -ENODEV;
    }

    //Enable V INT detect pin (GPIO Input Pin), read modem power status.
    int r1 = gpio_pin_configure(gpio_dev, V_INT_DETECT_GPIO_PIN,
                                GPIO_INPUT | V_INT_DETECT_GPIO_FLAGS);

    //Enable Serial Buffer Pin
    int r2 = gpio_pin_configure(gpio_dev, SERIAL_BUFFER_ENABLE_GPIO_PIN,
                                GPIO_OUTPUT_ACTIVE | SERIAL_BUFFER_ENABLE_GPIO_FLAGS);

    //Config Modem Power Pins
    const struct device *gpio_devX = device_get_binding(GPIO0);
    //Reset Pin
    int r3 = gpio_pin_configure(gpio_devX, SARA_RST_PIN,
                                GPIO_OUTPUT);
    //Reset Pin
    int r4 = gpio_pin_configure(gpio_devX, SARA_PWR_PIN,
                                GPIO_OUTPUT);

    return (r1 || r2 || r3 || r4);
}

/**
 * @brief Initialises the modem and power cycles it as per spec 
 *          to bring it upto operational standards.
 * 
 * @return int 
 */
int modem_pin_init(void)
{
    //Config Modem Power Pins
    const struct device *gpio_dev = device_get_binding(GPIO0);
    LOG_INF("Setting Modem Pins");

    LOG_DBG("MDM_RESET_PIN -> NOT_ASSERTED");

    gpio_pin_set(gpio_dev, SARA_RST_PIN, MDM_RESET_NOT_ASSERTED);

    LOG_DBG("MDM_POWER_PIN -> ENABLE");

    gpio_pin_set(gpio_dev, SARA_PWR_PIN, MDM_POWER_ENABLE);
    //Hardware is timing specific, required to wait.
    k_sleep(K_SECONDS(4));

    LOG_DBG("MDM_POWER_PIN -> DISABLE");
    //Begins Sara-R4 Power Sequence
    gpio_pin_set(gpio_dev, SARA_PWR_PIN, MDM_POWER_DISABLE);

    k_sleep(K_SECONDS(4));

    LOG_DBG("MDM_POWER_PIN -> ENABLE");
    gpio_pin_set(gpio_dev, SARA_PWR_PIN, MDM_POWER_ENABLE);

    k_sleep(K_SECONDS(1));

    LOG_DBG("Waiting for MDM_VINT_PIN = 0");

    //Read VINT to detect Modem Power Status
    while (gpio_pin_get(gpio_dev, SARA_VINT_PIN) > 0)
    {
        k_sleep(K_MSEC(100));
    }
    LOG_DBG("MDM_POWER_PIN -> DISABLE");

    unsigned int irq_lock_key = irq_lock();

    gpio_pin_set(gpio_dev, SARA_PWR_PIN, MDM_POWER_DISABLE);

    k_sleep(K_SECONDS(1));

    irq_unlock(irq_lock_key);

    LOG_DBG("MDM_POWER_PIN -> ENABLE");

    LOG_DBG("Waiting for MDM_VINT_PIN = 1");

    do
    {
        LOG_DBG("Waiting...");
        k_sleep(K_MSEC(100));
    } while (gpio_pin_get(gpio_dev, SARA_VINT_PIN) == 0);

    //R4 Zephyr Driver Does this for some reason.
    gpio_pin_configure(gpio_dev, SARA_PWR_PIN, GPIO_INPUT);

    LOG_INF("... Done!");
    return 0;
}

//static const char *poll_data = "AT+CGMI\r";

void modem_uart_tx(char *command)
{
    printk("Sent: %s\n", command);
    const struct device *uart_dev = device_get_binding(UART1);
    /* Verify uart_poll_out() */
    for (int i = 0; i < strlen(command); i++)
    {
        uart_poll_out(uart_dev, command[i]);
    }
}
