/**
 ************************************************************************
 * @file tsd10_adc.h
 * @author Wilfred MK
 * @date 13.05.2021 (Last Updated)
 * @brief tsd10_adc driver
 **********************************************************************
 **/

#ifndef TSD10_ADC_H
#define TSD10_ADC_H

#define ADC DT_LABEL(DT_NODELABEL(adc))
#define GPIO0 DT_LABEL(DT_NODELABEL(gpio0))
#define ADC_CHANNEL_0 0

//!
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_1ST_CHANNEL_ID 0                                    //Map to pin A0
#define ADC_1ST_CHANNEL_INPUT SAADC_CH_PSELP_PSELP_AnalogInput1 //This was found through trial and error

#define BUFFER_SIZE 1
//!
/* ==================================================================== */
/* ===========================THREAD DEFINES=========================== */
/* ==================================================================== */
/* Debug Thread Stack size */
#define STACK_SIZE_TSD_THREAD 512
/* Debug Thread Priority */
// Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible
#define THREAD_PRIORITY_TSD_THREAD 7

/* ==================================================================== */
/* ==============================THREADS=============================== */
/* ==================================================================== */
void thread_tsd10_adc(void);

#endif