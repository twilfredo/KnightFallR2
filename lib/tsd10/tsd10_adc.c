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

/* ADC Channel 0 Config Struct */
const struct adc_channel_cfg channel_cfg = {
    .channel_id = ADC_CHANNEL_0,
    .reference = ADC_REF_INTERNAL,
    .gain = ADC_GAIN_1,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
    .differential = 0,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + ADC_CHANNEL_0,
#endif
};

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

    static int16_t sample_buffer;

    const struct adc_sequence sequence = {
        /* individual channels will be added below */
        .channels = BIT(ADC_CHANNEL_0),
        .buffer = &sample_buffer,
        .oversampling = 4,
        /* buffer size in bytes, not number of samples */
        .buffer_size = sizeof(sample_buffer),
        .resolution = 14,
        .calibrate = true,
        .options = NULL,
    };

    while (1)
    {
        // if (adc_read(adc_dev, &sequence) < 0)
        // {
        //     LOG_ERR("Error reading TSD10 ADC Channel");
        // }
        // //TODO Able to read RAW Values, Add convertion to volts.
        // //TODO Tune ADC Reads Vals.
        //TODO Reading ADC affects modem init (Use sem to only read ADC when modem isn't busy powering)
        // //printk("Read: %d\n", sample_buffer);
        k_msleep(5000);
    }
}