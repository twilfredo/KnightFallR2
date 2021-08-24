
/**
 ************************************************************************
 * @file sensors_custom.h
 * @author Wilfred MK
 * @date 15.05.2021 (Last Updated)
 * @brief Sensor module to interact with the I2C sensor bus. 
 **********************************************************************
 **/

#ifndef SENSORS_CUSTOM_H
#define SENSORS_CUSTOM_H

#define STACK_SIZE_SENSORS 2048
#define THREAD_PRIORITY_SENSORS 7

struct sensor_data
{
    float ambientTemperature, pitch, roll;
};

extern struct k_msgq sensor_msgq;
/* ==================================================================== */
/* ============================I2C-PORT0=============================== */
/* ==================================================================== */
/* Device tree node identifier for I2C0 */
#define I2C0 DT_NODELABEL(i2c0)

void thread_sensors(void);

int init_i2c(void);

#endif