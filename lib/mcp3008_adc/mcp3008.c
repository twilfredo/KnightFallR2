#include <zephyr.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <drivers/spi.h>

#include "mcp3008.h"
#include "sensor_ctrl.h"
/* Message queue for sensor to upper abstraction layer */
K_MSGQ_DEFINE(tsd_msgq, sizeof(struct tsd_data), 10, 4);
/* Define logging module */
LOG_MODULE_REGISTER(MCP3008, LOG_LEVEL_INF);

/**
 * @brief Takes in an ADC reading in mV and return an NTU value that represents the approximate
 *          turbidity of a solution. Conversion equation is derived from the TSD-10 datasheet 
 *          curve fit. 
 * 
 * @note Approximated convertion equation dervived using parabolic curve fitting to the dataset
 *       NTU = -0.000000056367195797083(mV^3) + 0.000903132831336(mV^2) -5.26249626122612(mV) + 10611.2487902283
 *       R^2 = 0.9996820
 * 
 * @param mV value to the converted in to NTUs
 * @return int respective NTU value. 
 */
float millivolts_to_NTU(int mV)
{
    if (mV <= 1700) //Edge Case 1, Sensor Min Limit
        return 4000.00;

    if (mV >= 4600)
        return 0.00;

    double coef1 = -(0.000000056367195797083) * mV * mV * mV;
    double coef2 = 0.000903132831336 * mV * mV;
    double coef3 = -(5.26249626122612) * mV;
    double constant = 10611.2487902283;

    return (coef1 + coef2 + coef3 + constant);
}

float adc_to_mV(uint16_t adcRead)
{
    return ((adcRead * MCP3008_REF) / MCP3008_RESOLUTION) * 1000.00;
}

float adc_to_voltage(uint16_t adcRead)
{
    return ((adcRead * MCP3008_REF) / MCP3008_RESOLUTION);
}

/**
 * @brief Initialises SPI, and polls the MCP3008 for an ADC reading.
 * 
 * @note The following has be created to match specifications staed by the 
 *          MCP3008 datasheet, things might seems a bit weird. 
 * 
 */
void thread_adc_ctrl(void *p1, void *p2, void *p3)
{

    /* Create SPI Conf */
    const struct device *spi = device_get_binding(DT_LABEL(SPI));

    struct spi_config spi_cfg = {0};

    struct spi_cs_control *ctrl =
        &(struct spi_cs_control){
            .gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
            .delay = SPI_CS_DELAY,
            .gpio_pin = SPI_CS_PIN,
            .gpio_dt_flags = GPIO_ACTIVE_LOW};

    spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8);
    spi_cfg.frequency = MCP3008_MAX_CLK;
    spi_cfg.slave = MCP3008_SLAVENUM;
    spi_cfg.cs = ctrl;

    /* tx_data should be in this format, as per data sheet */
    uint8_t tx_data[3] = {MCP3008_LEADING_BYTE, MCP3008_SINGLE_ENDED_CH0, 0x00};
    uint8_t rx_data[3] = {0x00};

    struct spi_buf bufs_out[] =
        {
            {.buf = tx_data,
             .len = sizeof tx_data}};

    struct spi_buf bufs_in[] =
        {
            {.buf = rx_data,
             .len = sizeof tx_data}};

    struct spi_buf_set tx = {
        .buffers = bufs_out};

    struct spi_buf_set rx = {
        .buffers = bufs_in};

    int err = 0;
    uint16_t adcVal = 0;
    float adcVoltage = 0;
    float adcVoltageAvg = 0;

    struct tsd_data tsdData = {0};

    while (1)
    {
        adcVoltageAvg = 0;
        /* Wait until poll ok */
        k_sem_take(&tsd10_read_sem, K_FOREVER);

        /* Take 10 samples and calculate the average */
        LOG_INF("Reading TSD-10...");
        for (int i = 1; i <= ADC_SAMPLES; ++i)
        {
            err = spi_transceive(spi, &spi_cfg, &tx, &rx);

            if (err)
            {
                LOG_ERR("SPI Tranceive failed, errno: %d", err);
            }

            adcVal = ((uint16_t)(rx_data[1] & 0x03) << 8) | rx_data[2];
            adcVoltage = adc_to_mV(adcVal); //In mV
            adcVoltageAvg += adcVoltage;    //In mV

            //printk("Data: %d, %fV, %fmV\n", adcVal, adcVoltage, adc_to_mV(adcVal));
            /* Take samples not entirely close to each other, but not too far apart */
            k_msleep(50);
            /* Reset RX Data */
            memset(rx_data, 0, sizeof(rx_data));
        }

        adcVoltageAvg /= ADC_SAMPLES;

        tsdData.tsd_mV = adcVoltageAvg;
        tsdData.tsd_NTU = millivolts_to_NTU(adcVoltageAvg);

        LOG_INF("Measured: %d NTUs", (int)tsdData.tsd_NTU);

        /* Send this reads data to queue */
        if (k_msgq_put(&tsd_msgq, &tsdData, K_NO_WAIT) != 0)
        {
            LOG_WRN("Purging TSD-10 msgq...");
            k_msgq_purge(&tsd_msgq); //Make Space
        }

        // printk("NTUs: %f\n", millivolts_to_NTU(adcVoltageAvg));
        // printk("TSD-10 Voltage: %fmV\n", adcVoltageAvg);

        //Clear current data, queue is pass by copy not reference
        memset(&tsdData, 0, sizeof(struct tsd_data));

        k_msleep(500);
    }
}
