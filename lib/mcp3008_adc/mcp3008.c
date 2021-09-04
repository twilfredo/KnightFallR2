#include <zephyr.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <drivers/spi.h>

#include "mcp3008.h"
#include "sensor_ctrl.h"

LOG_MODULE_REGISTER(MCP3008, LOG_LEVEL_INF);

/**
 * @brief Takes in an ADC reading in mV and return an NTU value that represents the approximate
 *          turbidity of a solution. Conversion equation is derived from the TSD-10 datasheet 
 *          curve fit. 
 * 
 * @note Approximated convertion equation dervived using parabolic curve fitting to the dataset
 *       f(x) = 0.000314577x^2 − 2.85578x + 6202.72
 * 
 * @param mV value to the converted in to NTUs
 * @return int respective NTU value. 
 */
float millivolts_to_NTU(int mV)
{
    if (mV < 800)
    {
        //Edge Case 1, Sensor Limit
        return 4000.0;
    }

    if (mV > 4700)
    {
        //Edge Case 2, MAX ADC
        return 0.0;
    }

    float coef1 = 0.000314577 * mV * mV;
    float coef2 = 2.85578 * mV;

    return coef1 - coef2 + 6202.72;
}

float adc_to_mV(uint16_t adcRead)
{
    return ((adcRead * MCP3008_VDD) / MCP3008_RESOLUTION) * 1000.00;
}

float adc_to_voltage(uint16_t adcRead)
{
    return ((adcRead * MCP3008_VDD) / MCP3008_RESOLUTION);
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

        k_poll_signal_raise(&tsd10_sig, millivolts_to_NTU(adcVoltageAvg));

        //printk("NTUs: %f\n", millivolts_to_NTU(adcVoltageAvg));
        //printk("TSD-10 Voltage: %fmV\n", adcVoltageAvg);

        k_msleep(500);
    }
}
