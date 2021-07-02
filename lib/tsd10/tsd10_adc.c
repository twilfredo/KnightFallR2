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

/* Local Includes */
#include "tsd10_adc.h"
#include "sensor_ctrl.h"

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
    //TODO ADC Deinit Function
    const struct adc_sequence sequence = {
        .channels = BIT(ADC_1ST_CHANNEL_ID),
        .buffer = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
    };
    int32_t mvVal = 0;

    while (1)
    {
        /* Taking this sem allows for the adc to aquire a reading */
        if (k_sem_take(&tsd10_read_sem, K_FOREVER) == 0)
        {
            if (adc_read(adc_dev, &sequence) < 0)
            {
                LOG_ERR("Error reading TSD10 ADC Channel");
            }
            //TODO Reading ADC affects modem init (Use sem to only read ADC when modem isn't busy powering)- TESTED OK NOW (?)
            //TODO Scale TSD-10 Voltage 4.7Vpk to 3.6vPk (Input Max is 3.6 given 600mv/(1/6) = 3.6V [Vref/gain]
            //TODO Add voltage mV conversion and update the signal with NTUs.
            mvVal = sample_buffer[0];
            adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &mvVal);
            //printk("Raw: %d\n", sample_buffer[0]);
            //printk("Sent: %dmv\n", mvVal); //Print Voltage in mV
            k_poll_signal_raise(&tsd10_sig, millivolts_to_NTU(mvVal));
            //printk("NTUs: %0.2f\n", millivolts_to_NTU(mvVal));
        }
    }
}

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

    if (mV > 3600)
    {
        //Edge Case 2, MAX ADC
        return 0.0;
    }
    float coef1 = 0.000314577 * mV * mV;
    float coef2 = 2.85578 * mV;

    return coef1 - coef2 + 6202.72;
}