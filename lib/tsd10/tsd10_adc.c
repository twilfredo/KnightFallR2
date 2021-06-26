/**
 ************************************************************************
 * @file tsd10_adc.c
 * @author Wilfred MK
 * @date 13.05.2021 (Last Updated)
 * @brief tsd10_adc driver
 **********************************************************************
 **/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/adc.h>
#include <logging/log.h>
#include "tsd10_adc.h"

/* Register Log Module */
LOG_MODULE_REGISTER(TSD10_ADC, LOG_LEVEL_DBG);

static const struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_1ST_CHANNEL_ID,
    .input_positive = ADC_1ST_CHANNEL_INPUT,
};

static int16_t sample_buffer[BUFFER_SIZE];

/**
 * @brief Initialises ADC0 and 
 *          Polls measurements from the TSD10 sensor.
 * 
 */
void thread_tsd10_adc(void)
{
    const struct device *adc_dev = device_get_binding(ADC);

    /* Initialise ADC channel prior to read */
    if (adc_channel_setup(adc_dev, &channel_cfg) < 0)
    {
        LOG_ERR("Failed to setup ADC Channel");
    }

    const struct adc_sequence sequence = {
        .channels = BIT(ADC_1ST_CHANNEL_ID),
        .buffer = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
    };
    int32_t mvVal = 0;
    while (1)
    {
        if (adc_read(adc_dev, &sequence) < 0)
        {
            LOG_ERR("Error reading TSD10 ADC Channel");
        }
        //TODO Reading ADC affects modem init (Use sem to only read ADC when modem isn't busy powering)- TESTED OK NOW (?)
        //TODO Scale TSD-10 Voltage 4.7Vpk to 3.6vPk (Input Max is 3.6 given 600mv/(1/6) = 3.6V [Vref/gain]
        mvVal = sample_buffer[0];
        adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &mvVal);
        //printk("Raw: %d\n", sample_buffer[0]);
        printk("Read: %dmv\n", mvVal); //Print Voltage in mV
        k_msleep(500);
    }
}