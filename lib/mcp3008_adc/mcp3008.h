#ifndef MCP3008_H
#define MCP3008_H

/* Debug Thread Stack size */
#define STACK_SIZE_ADC_THREAD 2048
/* Debug Thread Priority */
#define THREAD_PRIORITY_ADC 5 /* Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible  */
/* MCP DEFINES */
#define MCP3008_VDD 4.70        //Volts - Calibrate as requried
#define MCP3008_RESOLUTION 1023 //10-Bit Resolution
#define MCP3008_MAX_CLK 1350000 //Hz
#define MCP3008_SLAVENUM 0
#define MCP3008_SINGLE_ENDED_CH0 0x80
#define MCP3008_LEADING_BYTE 0x01
#define ADC_SAMPLES 5
/* SPI CONFIG */
#define SPI_CS_PIN 31
#define SPI_CS_DELAY 2
#define SPI DT_NODELABEL(spi3)

void thread_adc_ctrl(void);

float adc_to_mV(uint16_t adcRead);

float adc_to_voltage(uint16_t adcRead);

float millivolts_to_NTU(int mV);

#endif
