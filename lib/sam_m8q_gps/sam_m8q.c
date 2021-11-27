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
#include <string.h>
/* Local Includes */
#include "sam_m8q.h"
#include "sensor_ctrl.h"
#include "sensor_pwr.h"

LOG_MODULE_REGISTER(SAM_M8Q, LOG_LEVEL_INF);
/* Inline Prototype */
static inline void sam_m8q_uart_tx(uint8_t *data, int arrSize);

/* UART RING BUFFERS */
#define RING_BUF_SIZE (64 * 2)
uint8_t rb_buf_sam[RING_BUF_SIZE];
struct ring_buf rx_rb_sam;

/* SAM Send/Rec Semaphore */
K_SEM_DEFINE(samRecSem, 0, 1);

/* GLL Data Packet Message Queue, Sends data to sensor control */
K_MSGQ_DEFINE(gps_msgq, sizeof(struct samGLLMessage), 10, 4);

/* Notifies the GPS polling thread to stop polling */
bool gpsTimeOutOccured = false;

/* Config Data Array: UBX Protocol - From uCentre*/
// Disable NMEA String Outputs
static uint8_t cmd_ggaOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
//static uint8_t cmd_gllOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
static uint8_t cmd_gsaOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
static uint8_t cmd_gsvOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
static uint8_t cmd_rmcOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
static uint8_t cmd_vtgOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
//UBX Protocol Off
static uint8_t cmd_pvtOff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC};
//Set GPS output rate
//static uint8_t cmd_updateRate5Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
static uint8_t cmd_updateRate1Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};

/**
 * @brief Config SAM_M8Q module to only output data using the NMEA protocol, and to disable the typical
 *          additional NMEA strings except for the GNGLL string which contains the followings data. 
 *        Field 0: xxGLL Message ID
 *        Field 1: Lat
 *        Field 2: N/S Indicator
 *        Field 3: Long
 *        Field 4: E/W Indicator
 *        Field 5: UTC Time 
 *        Field 6: Status "V" data invalid of receiver warning, "A" = Data valid.
 *        Sets data poll rate to 5Hz
 * 
 *      That is, once this config is set, the module should only return string of the aforementioned structure.
 * 
 * @note The delays must be kept to allow for the module to read properly, otherwise risk exceeding input baudrate for 
 *       SAM-M8Q
 */
void sam_m8q_config(void)
{
    /* Turn GGA Off */
    sam_m8q_uart_tx(cmd_ggaOff, sizeof cmd_ggaOff);
    k_msleep(SAM_CMD_DELAY);
    /* Turn GSA Off */
    sam_m8q_uart_tx(cmd_gsaOff, sizeof cmd_gsaOff);
    k_msleep(SAM_CMD_DELAY);
    /* Turn GSV Off */
    sam_m8q_uart_tx(cmd_gsvOff, sizeof cmd_gsvOff);
    k_msleep(SAM_CMD_DELAY);
    /* Turn RMC Off */
    sam_m8q_uart_tx(cmd_rmcOff, sizeof cmd_rmcOff);
    k_msleep(SAM_CMD_DELAY);
    /* Turn VTG Off */
    sam_m8q_uart_tx(cmd_vtgOff, sizeof cmd_vtgOff);
    k_msleep(SAM_CMD_DELAY);
    /* Turn PVT Off */
    sam_m8q_uart_tx(cmd_pvtOff, sizeof cmd_pvtOff);
    k_msleep(SAM_CMD_DELAY);
    /* Set Update Rate */
    sam_m8q_uart_tx(cmd_updateRate1Hz, sizeof cmd_updateRate1Hz);
    k_msleep(SAM_CMD_DELAY);
}

/**
 * @brief Controls and sends data to the SAM-M8Q GPS module. 
 * 
 */
void thread_gps_ctrl(void *p1, void *p2, void *p3)
{

    if (sam_m8q_uart_init())
    {
        LOG_ERR("UART Config Error");
    }

    /* Power on GPS, So it can acquire lock while system is setting up. This can be removed, but it speeds up the GPS lock process on a cold start. 
     * This increases the concurrent current draw during the boot up sequence. 
     */
    sam_m8q_pwr_on();
    k_msleep(VRAIL_DELAY);
    //Configure the module to send only GLL (Needs upon power cycle)
    sam_m8q_config();

    struct samGLLMessage gllMsgPacket = {0};

    while (1)
    {
        //1. Wait on activation semaphore
        k_sem_take(&gps_read_sem, K_FOREVER);
        //2. Configure the module to send only GLL (Needs upon power cycle)
        sam_m8q_config();
        //3. Poll GPS Data for a lock.
        do
        {
            //4. Wait of recv ok sem
            k_sem_take(&samRecSem, K_SECONDS(SAM_TIMEOUT));

            /* Is GPS Message Valid? */
            if (sam_recv(&gllMsgPacket) == true)
            {
                /* GPS Positioning aquired */
                // 5. Send data back to sensor control thread
                if (k_msgq_put(&gps_msgq, &gllMsgPacket, K_NO_WAIT) != 0)
                {
                    k_msgq_purge(&gps_msgq);
                    LOG_ERR("Unable to dispatch GPS data to sensor_ctrl");
                }
                break; //Break the for loop, message sent
            }
            /* Parse until a valid packet is recvd */
        } while (gpsTimeOutOccured == false);

        gpsTimeOutOccured = false;
        //Clear current data, queue is pass by copy not reference
        memset(&gllMsgPacket, 0, sizeof gllMsgPacket);
    }
}

/**
 * @brief Receive data from sam gps ring buffer
 * 
 * @param gllMsgPacket data struct to hold parsed lon/lat info into.
 * @return true if successfully parsed and saved into gllMsgPacket
 * @return false parsing failed
 */
bool sam_recv(struct samGLLMessage *gllMsgPacket)
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

    //TODO: Copy to a struct
    if (sam_recv_parse(gllMsgPacket, buffer, sizeof buffer))
    {
        /* Valid message parsed, data saved */
        return true;
    }

    return false;
}

/**
 * @brief Parse and copy received gps data value to the passed in data struct
 * 
 * @param gllMsgPacket to save gll message contents into
 * @return true if the received message was valid, dictated by field 6 status value 'A', dead reckoning fix. 
 * @return false received message was invalid.
 */
bool sam_recv_parse(struct samGLLMessage *gllMsgPacket, char *gllMsg, int sizeofBuffer)
{
    //printk("%s\n\r", gllMsg);

    char latBuffer[64];
    char latInd[64];
    char lonBuffer[64];
    char lonInd[64];
    char status[64];

    char *slice = strtok(gllMsg, ",");
    /* Case counter represents tokens (fields) split */
    int caseCounter = 1;

    /* The gllMsg string is formatted a specific way as documented by the the ublox NMEA protocol manual */
    while (slice != NULL)
    {
        switch (caseCounter)
        {
        case 2:
            /* Latitude : Field 2 */
            strcpy(latBuffer, slice);
            break;
        case 3:
            /* Latitude Indicator : Field 3*/
            strcpy(latInd, slice);
            break;
        case 4:
            /* Longitude : Field 4 */
            strcpy(lonBuffer, slice);
            break;
        case 5:
            /* Longitude Indicator : Field 5*/
            strcpy(lonInd, slice);
            break;
        case 7:
            /* Message Validity Indicator : Field 7 */
            strcpy(status, slice);
            break;
        }

        slice = strtok(NULL, ",");
        caseCounter++;
    }

    /* Debug */
    //printk("2. %s\n", latBuffer);
    //printk("3. %s\n", latInd);
    //printk("4. %s\n", lonBuffer);
    //printk("5. %s\n", lonInd);
    //printk("7. %s\n", status);

    if (caseCounter == 9 && !strcmp(status, "A"))
    {
        /* Correct msgLength and Status field is A (Valid) - DeadReckoning Fix */
        //TODO: Reading Attained, can be power
        //printk("Lat: %2f\n", GpsToDecimalDegrees(latBuffer, latInd[0]));
        gllMsgPacket->lat = GpsToDecimalDegrees(latBuffer, latInd[0]);
        //printk("Lon: %2f\n", GpsToDecimalDegrees(lonBuffer, lonInd[0]));
        gllMsgPacket->lon = GpsToDecimalDegrees(lonBuffer, lonInd[0]);
        return true;
    }

    return false;
}

/**
 * @note This function is a generic function attained from stackoverflow, and is used to convert NMEA absolute positions to decimal degrees. 
 * 
 * @brief Convert NMEA absolute position to decimal degrees
 *          "ddmm.mmmm" or "dddmm.mmmm" really is D+M/60,
 *          then negated if quadrant is 'W' or 'S'
 * 
 * @ref https://stackoverflow.com/questions/36254363/how-to-convert-latitude-and-longitude-of-nmea-format-data-to-decimal
 * 
 * @param nmeaPos as string 
 * @param quadrant N/S, E/W
 * @return double decimal degree from NMEA absolute position. 
 */
double GpsToDecimalDegrees(const char *nmeaPos, char quadrant)
{
    double v = 0;
    if (strlen(nmeaPos) > 5)
    {
        char integerPart[3 + 1];
        int digitCount = (nmeaPos[4] == '.' ? 2 : 3);
        memcpy(integerPart, nmeaPos, digitCount);
        integerPart[digitCount] = 0;
        nmeaPos += digitCount;
        v = atoi(integerPart) + atof(nmeaPos) / 60.;
        if (quadrant == 'W' || quadrant == 'S')
            v = -v;
    }
    return v;
}

/**
 * @brief Sends uart commands to sam_m8q device on uart0 TX.
 * 
 * @param data array of bytes of data to send.
 * @param arrSize size of array in bytes. 
 */
static inline void sam_m8q_uart_tx(uint8_t *data, int arrSize)
{
    const struct device *uart0_dev = device_get_binding(UART0);

    for (int i = 0; i < arrSize; ++i)
    {
        uart_poll_out(uart0_dev, data[i]);
    }
}

/**
 * @brief Callback function to read data from the modem, and save bytes to allocated
 *          ring buffer. 
 * 
 * @param uart_device 
 * @param user_data 
 */
static void
uart_cb(const struct device *uart_device, void *user_data)
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
                LOG_DBG("Rx buffer doesn't have enough space. "
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