#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include "sensors_custom.h"
#include "mcp9808.h"
#include "adxl343.h"
#include <math.h>

LOG_MODULE_REGISTER(sensor_driver, LOG_LEVEL_DBG);
K_THREAD_DEFINE(sensor_driver, STACK_SIZE_SENSORS, thread_sensors, NULL, NULL, NULL, THREAD_PRIORITY_SENSORS, 0, 10);

/* Defines and initializes an empty message queue that is capable of holding 10 items */
K_MSGQ_DEFINE(sensor_msgq, sizeof(struct sensor_data), 10, 4);

/**
 * @brief Initialise I2C Bus and read data from all sensors when requried. 
 * 
 */
void thread_sensors(void)
{
    //Init I2C Bus
    init_i2c();
    //Validate MCP9808 on Bus
    mcp9808_verify();
    //Validate ADXL343 on Bus
    adxl343_verify();
    //Set ADXL measurement mode, and 4g
    adxl343_set_measurement_mode();

    //MessageQ Data Buffer
    struct sensor_data sensorData = {0};

    while (1)
    {

        //Update MessageQ Buffers
        sensorData.ambientTemperature = mcp9808_read_ambient_temp();
        sensorData.roll = adxl343_get_roll();
        sensorData.pitch = adxl343_get_pitch();

        while (k_msgq_put(&sensor_msgq, &sensorData, K_NO_WAIT) != 0)
        {
            /* message queue is full: purge old data & try again */
            k_msgq_purge(&sensor_msgq);
        }

        //printk("Tilt: %.2f, Pitch: %.2f\n\r", sensorData.roll, sensorData.pitch);
        // printk("x:%d y:%d z:%d \n", adxlXYZBuffer[0], adxlXYZBuffer[1], adxlXYZBuffer[2]);
        // printk("Temperature: %f", mcp9808_read_ambient_temp());
        k_msleep(100);
    }
}

/**
 * @brief Initialise the I2C bus
 * 
 * @return 0 on success, ERRVAL else.
 */
int init_i2c(void)
{
    const struct device *dev_i2c0 = device_get_binding(DT_LABEL(I2C0));
    if (!dev_i2c0)
    {
        /* Unable to retrive device structure */
        return -ENODEV;
    }
    else
    {
        /*Configure I2C using attained device struct, using the following parameters */
        int err = i2c_configure(dev_i2c0, I2C_SPEED_FAST | I2C_MODE_MASTER);
        return err;
    }
}
