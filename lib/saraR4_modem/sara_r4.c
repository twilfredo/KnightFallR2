/**
 ************************************************************************
 * @file sara_r4.c
 * @author Wilfred MK
 * @date 13.05.2021 (Last Updated)
 * @brief Custom sara-r4 communication module. This module abstracts 
 *          sara-r4 driver and perform network management. 
 *        Sub-routines include
 *                  1. Power cycling the modem (Timing Critical)
 *                  2. Set Sara R4 in HTTP Mode
 *                  3. HTTP Get requests to send sensor data, and get Config data
 *                  4. Sleeping Modem
 *@note This is not a standalone driver for the sara R4 Modem, it integrates
            the networking and the modem for the specific usecase of this project. 
 ************************************************************************
 **/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/ring_buffer.h>

#include "sensor_ctrl.h"
#include "sara_r4.h"
#include "dbg_led.h"
#include "jsmn.h"

LOG_MODULE_REGISTER(SARA_R4, LOG_LEVEL_DBG);

/* UART RING BUFFER */
#define RING_BUF_SIZE (2048)
uint8_t rb_buf[RING_BUF_SIZE];
struct ring_buf rx_rb;

/* SENSOR DATA STRING BUFFERS */
char turbidity[SD_LEN];
char longitude[SD_LEN];
char lattitude[SD_LEN];
char dataPacket[64];

/* Modem Send/Rec Semaphore */
K_SEM_DEFINE(modemSendSem, 1, 1);
K_SEM_DEFINE(modemRecSem, 0, 1);
K_SEM_DEFINE(modemReadOkSem, 0, 1);
K_SEM_DEFINE(modemCommandOkSem, 0, 1);
K_SEM_DEFINE(networkReady, 0, 1);
K_SEM_DEFINE(modemGetSettings, 0, 1);

/* NETWORK CONFIG */
#define MODEM_APN "telstra.internet"
#define MODEM_MCCMNO "50501"
#define AT_INIT_CMD_SIZE 9 //Num elements in atInitCommands

/* RECV THREAD DELAY */
#define RX_READ_DELAY_SECS 1

/* Semaphore Timeouts */
#define RX_RESPONSE_TIMEOUT_SECS 30 //Timeout for modem rx OK
#define INIT_SEND_TIMEOUT_S 60      //Waiting duration for modem init commands
#define SEND_TIMEOUT 30             //Waiting duration for sending data to modem (Semphore)
#define AT_OK_TIMEOUT 30            //Waiting duration for an OK message from modem

/* HTTP to ThingSpeak */
#define HTTP_INIT_CMD_SIZE 3 //Num elements in httpSetupCommands
#define HTTP_TIMEOUT 30
#define TS_HTTP_ADDR "api.thingspeak.com"
#define READ_FILE "read.rsp"
#define SET_FILE_NAME "set.rsp"
/* A numeric value followed by '=' will be set into that respective field */
#define UPDATE_ADDR_OUT "/update?api_key=94Z2J4FS3282TET3&field1="
/* Read last config numeric from field8 */
#define READ_ADDR "/channels/1501295/fields/8/last.json"

//TODO 1. Modem Sleep Function -> CFUN0
//TODO 2. Modem Wake Function   -> CFUN1 i think
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
    "ATE0\r",
    "AT+CFUN=0\r",
    "AT+CMEE=1\r",
    "AT+UGPIOC=23,0,0\r",
    "AT+CREG=1\r",                             //5
    "AT+CGDCONT=1,\"IP\",\"" MODEM_APN "\"\r", //6
    "AT+CFUN=1\r",                             //7
    "AT+COPS=1,2,\"" MODEM_MCCMNO "\"\r",      //8
    "AT+CFUN=0\r"};                            //9

char httpSetupCommands[HTTP_INIT_CMD_SIZE][128] = {
    "AT+CFUN=1\r",
    "AT+UHTTPC=?\r",
    "AT+UHTTP=0,1, \"" TS_HTTP_ADDR "\"\r"};

// "AT+UHTTPC=0,1,\"" READ_ADDR "\" ,\"" READ_FILE "\"\r",
// "AT+URDFILE=\"" READ_FILE "\"\r"};

/**
    SEND GET       "AT+UHTTPC=0,1,\"" UPDATE_ADDR "\" ,\"" SET_FILE "\"\r",
    REC GET         "AT+UHTTPC=0,1,\"" READ_ADDR "\" ,\"" READ_FILE "\"\r",
    READ GET RESP   "AT+URDFILE=\"" READ_FILE "\"\r"};
 * 
 */

#define POLL_CMDS_SIZE 2
#define HTTP_GET_REQ_WAIT 10 /* In seconds the max expected time for GET request to resolve */

char pollCommands[POLL_CMDS_SIZE][128] = {
    "AT+UHTTPC=0,1,\"" READ_ADDR "\" ,\"" READ_FILE "\"\r",
    "AT+URDFILE=\"" READ_FILE "\"\r"};

/**
 * @brief Primary thread that communicates with the Sara-R4 Modem
 *          the sequence of operations in this thread must be maintained
 *          to ensure modem functionality. Do not change the timing. 
 * @note Whenever this thread gets a packet from the main thread, it will instantly try to send it to thingspeak. Therefore send delays will be controlled by
 *          SYS_ACTIVE_DELAY in the main thread. Limit this to > 15 Seconds. 
 */
void thread_modem_ctrl(void *p1, void *p2, void *p3)
{
    int packetsDropped = 0;

//Begin modem power sequnce
restart_modem:
    httpOk = false;
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
    LOG_INF("Registering on Network...");
    if (!modem_network_init())
    {
        LOG_WRN("Error Initializing Network, Restart Modem");
        //Unable Modem R/W
        k_sem_give(&modemSendSem);
        k_sem_give(&modemRecSem);
        goto restart_modem;
    }

    short connectionAttempts = 0;
//Establish connection to ThingSpeak
reconnect_HTTP:
    LOG_INF("Setting up HTTP...");

    packetsDropped = 0; //Reset dropped packet counts
    httpOk = false;

    if (!modem_http_init())
    {
        LOG_WRN("Error setting up HTTP");
        //Unable Modem R/W
        k_sem_give(&modemSendSem);
        k_sem_give(&modemRecSem);
        connectionAttempts++;

        if (connectionAttempts >= 5)
        {
            //Multiple Connection attempst failed, powercycle modem
            LOG_ERR("Too Many Connection Attempts, Restarting Modem");
            goto restart_modem;
        }
        else
        {
            //Attemp to recoonect
            goto reconnect_HTTP;
        }
    }

    httpOk = true;                            //Control DBG_LED_COLOUR
    char sendBuffer[128];                     //UART Packet sent to modem
    struct sensor_packet sensorDataRec = {0}; //Data Packet from the sensors

    LOG_INF("Network Ready...");

    while (1)
    {
        /* Indicate Network Ready */
        k_sem_give(&networkReady);

        /* Waits to receive sensor data from main thread */
        k_msgq_get(&to_network_msgq, &sensorDataRec, K_FOREVER); //Waits for sensors data to publish

        update_sensor_buffers(&sensorDataRec); //Updates sensor buffer (int to string)

        if (k_sem_take(&modemSendSem, K_SECONDS(SEND_TIMEOUT)) == 0)
        {
            /* Updates Turbidity Field on thingspeak */
            //This packet in the following form [turbidity#longitude#lattitude]
            //Field 1 is currently selected for packet streaming.
            snprintk(sendBuffer, 128, "AT+UHTTPC=0,1,\"" UPDATE_ADDR_OUT "%s\",\"" SET_FILE_NAME "\"\r", dataPacket);

            modem_uart_tx(sendBuffer);

            k_sem_give(&modemRecSem); //Singal modem recv thread

            if (k_sem_take(&modemCommandOkSem, K_SECONDS(AT_OK_TIMEOUT)) != 0)
            {
                //Modem response was not OK;
                LOG_WRN("GET Reqeust Send Error...");
                if (packetsDropped >= PCKTS_DROPPED_MAX)
                    goto reconnect_HTTP; //Reconnect if packets kept dropping.

                packetsDropped++;
            }
            else
            {
                LOG_INF("Message Sent OK...");

                /* Used to indicate message was parsed by the modem OK */
                turn_usr_led_on();
                k_msleep(50);
                turn_usr_led_off();
            }
        }
        else
        {
            //Unable to get send ok semaphore, attempt to re-establish connection to be safe.
            memset(&sensorDataRec, 0, sizeof sensorDataRec);
            goto reconnect_HTTP;
        }

        memset(&sensorDataRec, 0, sizeof sensorDataRec);
        memset(&sendBuffer, 0, sizeof sendBuffer);
    }
}

/**
 * @brief Receive data from the modem on UART RX
 *          calls modem_recv that will parse
 *          and log the received message from ring buffer.
 * 
 */
void thread_modem_receive(void *p1, void *p2, void *p3)
{

    while (1)
    {
        //Wait until a command is sent/
        if (k_sem_take(&modemRecSem, K_FOREVER) == 0)
        {
            //Wait until Rx Data is fully received.
            if (k_sem_take(&modemReadOkSem, K_SECONDS(RX_RESPONSE_TIMEOUT_SECS)) == 0)
            {
                //Wait for RX to complete reading.
                k_sleep(K_SECONDS(RX_READ_DELAY_SECS));
                //Read RX ISR Ring Buffer.
                if (modem_recv())
                {
                    //The received message was valid, i.e OK
                    k_sem_give(&modemSendSem);
                    k_sem_give(&modemCommandOkSem);
                }
                /* This semaphore needs to be reset, so that next time modemReadSem will wait till it is released */
                k_sem_reset(&modemReadOkSem);
            }
            else
            {
                /* Timeout occured, rearm recv for recovery */
                k_sem_give(&modemSendSem);
                k_sem_give(&modemCommandOkSem);
                k_sem_reset(&modemReadOkSem);
            }
            //Next command can be sent now.
        }
    }
}

/**
 * @brief Attemps to get config data from thingspeak, sends an HTTP request from the modem,
 *          the req result is read after an appropriate delay, parsed and current config is updated.
 * 
 */
void thread_modem_poll_settings(void *p1, void *p2, void *p3)
{
    while (1)
    {
        /* Wait till signalled to download settings */
        if (k_sem_take(&modemGetSettings, K_FOREVER) == 0)
        {
            if (modem_poll_settings() == false)
            {
                LOG_WRN("Unable to complete HTTP request for getting configuration settings.");
            }
        }
    }
}

bool modem_poll_settings(void)
{
    LOG_INF("Downloading Operation Profile");
    for (int i = 0; i < POLL_CMDS_SIZE; ++i)
    {

        if (k_sem_take(&modemSendSem, K_SECONDS(HTTP_TIMEOUT)) == 0)
        {
            modem_uart_tx(pollCommands[i]);

            if (i == 0)                                /* Read REQ is sent, waiting for modem to get results */
                k_sleep(K_SECONDS(HTTP_GET_REQ_WAIT)); /* Delay for expected time to complete a HTTP request */

            k_sem_give(&modemRecSem);

            if (k_sem_take(&modemCommandOkSem, K_SECONDS(HTTP_TIMEOUT)) != 0)
            {
                //Modem response was not OK, or was timed out;
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
 * @brief Intialises the modem to send HTTP request
 * 
 * @return true if init complete OK
 * @return false init complete failed
 */
bool modem_http_init(void)
{
    for (int i = 0; i < HTTP_INIT_CMD_SIZE; ++i)
    {

        if (k_sem_take(&modemSendSem, K_SECONDS(HTTP_TIMEOUT)) == 0)
        {
            modem_uart_tx(httpSetupCommands[i]);

            // //TODO REMOVE THIS
            // if (i == 3)
            //     k_msleep(4000);

            k_sem_give(&modemRecSem);
            if (k_sem_take(&modemCommandOkSem, K_SECONDS(HTTP_TIMEOUT)) != 0)
            {
                //Modem response was not OK, or was timed out;
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
        if (k_sem_take(&modemSendSem, K_SECONDS(INIT_SEND_TIMEOUT_S)) == 0)
        {
            //Modem has received data, send next command
            modem_uart_tx(atInitCommands[i]);
            k_sem_give(&modemRecSem);

            if (k_sem_take(&modemCommandOkSem, K_SECONDS(INIT_SEND_TIMEOUT_S)) != 0)
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
 * @brief Send an AT command to the modem using the uart interface
 * 
 * @param command AT command to be sent.
 */
void modem_uart_tx(char *command)
{
    const struct device *uart_dev = device_get_binding(UART1);
    /* Verify uart_poll_out() */
    for (int i = 0; i < strlen(command); i++)
    {
        uart_poll_out(uart_dev, command[i]);
    }
    LOG_DBG("MODEM SENT:\n %s\n", log_strdup(command));
}

/**
 * @brief Finds a match for a given json key 
 * 
 * @param json jsonc string "{"user": "name"}" 
 * @param tok jsmn token array
 * @param s string to compare
 * @return 0 if a match is found, -1 if not found
 */
static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0)
    {
        return 0;
    }
    return -1;
}

/**
 * @brief Takes a thingspeak request jason string and looks for "field8" and returns its value as an int.
 * @note {"field8":"1234"}
 * 
 * @param json_string 
 * @return int value of field8, -1 if unable to parse
 */
int json_parse_field8(char *json_string)
{
    jsmn_parser p;
    jsmntok_t t[128];
    int r, field8_val_int;
    char field8_val_str[32];
    char *endptr;

    jsmn_init(&p);
    r = jsmn_parse(&p, json_string, strlen(json_string), t, sizeof(t) / sizeof(t[0]));

    /* Loop over all keys of the root object */
    for (int i = 1; i < r; i++)
    {
        if (jsoneq(json_string, &t[i], "field8") == 0)
        {
            /* Convert field8 value to a string */
            snprintk(field8_val_str, 32, "%.*s", t[i + 1].end - t[i + 1].start,
                     json_string + t[i + 1].start);
            /* Convert filed8 value string to an int */
            field8_val_int = strtol(field8_val_str, &endptr, 10);
            return field8_val_int;
        }
    }
    return -1;
}

/**
 * @brief @brief Read modem responses from ring buffer, and parse the messages
 * 
 * @return true AT Command OK 
 * @return false AT Command ERR
 */
bool modem_recv(void)
{
    int numReadBytes = 0; //Number of bytes read from ringbuffer
    int jsonStartIndex = 0, jsonEndIndex = 0;
    uint8_t ringBufferLoad[1024];
    char jsonBuffer[128];

    /* Read data from UART RX Ring Buffer */
    numReadBytes = ring_buf_get(&rx_rb, ringBufferLoad, 1024);
    ringBufferLoad[numReadBytes] = '\0';

    /* The data recvd from the modem is a HTTP request (yes, this is terrible design)
     *  parse the HTTP request and extract from JSON field what the profile in was.
     * The JSON parsing code has been 'burrowed' from JSMN library.
     */
    if (numReadBytes > 512)
    {
        /* HTTP REQUEST FILE */
        for (int j = 0; j < numReadBytes; ++j)
        {
            if (ringBufferLoad[j] == '{')
            {
                jsonStartIndex = j;
            }

            if (ringBufferLoad[j] == '}')
            {
                jsonEndIndex = j;
                break;
            }
        }

        /* Extract the JSON object from the HTTP Request */
        strncpy(jsonBuffer, ringBufferLoad + jsonStartIndex, (jsonEndIndex + 1) - jsonStartIndex);
        jsonBuffer[(jsonEndIndex + 1) - jsonStartIndex] = '\0';

        int operationProfile = json_parse_field8(jsonBuffer);
        LOG_DBG("Profile Value: %d", operationProfile);
        set_operation_profile(operationProfile);
    }

    LOG_DBG("MODEM RESPONSE: %s", log_strdup(ringBufferLoad));

    //Check if the modem response had 'OK' in it
    for (int i = 0; i < numReadBytes; ++i)
    {
        if (ringBufferLoad[i] == 'O' && ringBufferLoad[i + 1] == 'K')
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Set the system active delay based on the operation profile.
 * 
 * @param opProfile profile value to set specific system delay presets. 
 */
void set_operation_profile(int opProfile)
{
    switch (opProfile)
    {
    case 1:
        SYS_ACTIVE_DELAY_SEC = 30;
        LOG_INF("System Delay updated to 30s");
        break;
    case 2:
        SYS_ACTIVE_DELAY_SEC = 60;
        LOG_INF("System Delay updated to 60s");
        break;
    case 3:
        SYS_ACTIVE_DELAY_SEC = 120;
        LOG_INF("System Delay updated to 120s");
        break;
    default:
        SYS_ACTIVE_DELAY_SEC = 30;
        LOG_WRN("Invalid profile, Settings System Delay 30s");
        break;
    }
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
    const struct device *dev_uart1 = uart_device;

    int rx = 0, ret;
    uint8_t *dst;
    uint32_t partial_size = 0;
    uint32_t total_size = 0;

    ARG_UNUSED(user_data);

    /* get all of the data off UART as fast as we can */
    while (uart_irq_update(dev_uart1) &&
           uart_irq_rx_ready(dev_uart1))
    {
        if (!partial_size)
            partial_size = ring_buf_put_claim(&rx_rb, &dst, UINT32_MAX);

        rx = uart_fifo_read(dev_uart1, dst, partial_size);

        if (rx <= 0)
            continue;

        dst += rx;
        total_size += rx;
        partial_size -= rx;
    }

    ret = ring_buf_put_finish(&rx_rb, total_size);
    __ASSERT_NO_MSG(ret == 0);

    if (total_size > 0)
    {
        k_sem_give(&modemReadOkSem);
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
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

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
        ring_buf_init(&rx_rb, RING_BUF_SIZE, rb_buf);
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
 *          to bring it upto operational mode.
 * 
 * @note Needs to follow this specific sequence, and is timing 
 *          dependent as per data sheet. 
 * 
 * @return int 
 */
int modem_pin_init(void)
{
    //Config Modem Power Pins
    const struct device *gpio_dev = device_get_binding(GPIO0);
    LOG_INF("Initializing Modem Power");

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

    LOG_INF("Modem Power OK...");
    return 0;
}

/**
 * @brief Updates the dataPacket (globally defined) buffer that contain sensor/location data.
 *  
 *        The created buffer is to be of the following format delimited with '$$'
 *         [Turbid$$Long$$Latt]
 * @param sensorData ptr to a sensor packet containing data to update internal
 *          buffers with.
 */
void update_sensor_buffers(struct sensor_packet *sensorData)
{
    memset(dataPacket, 0, sizeof dataPacket);

    if (sensorData->longitude == GPS_NO_LOCK_VAL || sensorData->lattitude == GPS_NO_LOCK_VAL)
    {
        snprintk(dataPacket, sizeof dataPacket, "NTU:%d$$LATT:NO_LOCK$$LONG:NO_LOCK", sensorData->turbidity);
    }
    else
    {
        snprintk(dataPacket, sizeof dataPacket, "NTU:%d$$LATT:%f$$LONG:%f", sensorData->turbidity, sensorData->lattitude, sensorData->longitude);
    }
}
