#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include "sensors_custom.h"
#include "adxl343.h"
#include <math.h>

LOG_MODULE_REGISTER(adxl343driver, LOG_LEVEL_DBG);

/**
 * @brief Verify that the ADXL343 device is active on 
 *     i2c0 bus.
 * 
 * @return 0 on success. 
 */
int adxl343_verify(void)
{
    const struct device *i2cDev = device_get_binding(DT_LABEL(I2C0));
    uint8_t buffer;

    if (i2c_reg_read_byte(i2cDev, ADXL_DEV_ADDR, ADXL_DEVID_ADDR, &buffer))
    {
        LOG_ERR("ADXL Read Error");
        return -EIO;
    }

    if (buffer != ADXL_WHOAMI_VAL)
    {
        LOG_ERR("Unable to verify ADXL343");
        return -EIO;
    }

    return 0;
}

void adxl343_set_measurement_mode(void)
{
    //TODO add inactive auto sleep
    //Set measurement mode;
    const struct device *i2cDev = device_get_binding(DT_LABEL(I2C0));

    if (i2c_reg_write_byte(i2cDev, ADXL_DEV_ADDR, ADXL_POWER_CTL_ADDR, ADXL_MEASURE_4HZ))
    {
        LOG_ERR("Write error");
        return;
    }

    //Set G Range
    if (i2c_reg_write_byte(i2cDev, ADXL_DEV_ADDR, ADXL_DATA_FMT_ADDR, ADXL_DATA_RANGE_4G))
    {
        LOG_ERR("Write error");
        return;
    }
}

void adxl343_read_xyz(int16_t *bufferSave)
{
    const struct device *i2cDev = device_get_binding(DT_LABEL(I2C0));

    uint8_t buffer[6] = {0};

    if (i2c_burst_read(i2cDev, ADXL_DEV_ADDR, ADXL_READ_START_ADDR, buffer, sizeof(buffer)))
    {
        LOG_ERR("XYZ read error");
        return;
    }

    int16_t xVal, yVal, zVal;
    xVal = buffer[1] << 8; //MSB
    xVal = xVal | (buffer[0] & 0x00FF);

    yVal = buffer[3] << 8; //MSB
    yVal = yVal | (buffer[2] & 0x00FF);

    zVal = buffer[5] << 8; //MSB
    zVal = zVal | (buffer[4] & 0x00FF);

    //Invert Direction to match arrows
    zVal = zVal * -1;
    xVal = xVal * -1;
    yVal = yVal * -1;

    bufferSave[0] = xVal;
    bufferSave[1] = yVal;
    bufferSave[2] = zVal;
    //printk("x:%d y:%d z:%d \n", xVal, yVal, zVal);
}

/**
 * @brief Read XYZ and calulate and return pitch
 * 
 * @return pitch
 */
double adxl343_get_pitch(void)
{
    const struct device *i2cDev = device_get_binding(DT_LABEL(I2C0));

    uint8_t buffer[6] = {0};

    if (i2c_burst_read(i2cDev, ADXL_DEV_ADDR, ADXL_READ_START_ADDR, buffer, sizeof(buffer)))
    {
        LOG_ERR("XYZ read error");
        return -1111;
    }

    int16_t xVal, yVal, zVal;
    xVal = buffer[1] << 8; //MSB
    xVal = xVal | (buffer[0] & 0x00FF);

    yVal = buffer[3] << 8; //MSB
    yVal = yVal | (buffer[2] & 0x00FF);

    zVal = buffer[5] << 8; //MSB
    zVal = zVal | (buffer[4] & 0x00FF);

    //Invert Direction to match arrows
    zVal = zVal * -1;
    xVal = xVal * -1;
    yVal = yVal * -1;

    return atan2((-xVal), sqrt(pow(yVal, 2) + pow(zVal, 2))) * 57.3;
}

/**
 * @brief Read XYZ and calulate and return roll
 * 
 * @return pitch
 */
double adxl343_get_roll(void)
{
    const struct device *i2cDev = device_get_binding(DT_LABEL(I2C0));

    uint8_t buffer[6] = {0};

    if (i2c_burst_read(i2cDev, ADXL_DEV_ADDR, ADXL_READ_START_ADDR, buffer, sizeof(buffer)))
    {
        LOG_ERR("XYZ read error");
        return -1111;
    }

    int16_t xVal, yVal, zVal;
    xVal = buffer[1] << 8; //MSB
    xVal = xVal | (buffer[0] & 0x00FF);

    yVal = buffer[3] << 8; //MSB
    yVal = yVal | (buffer[2] & 0x00FF);

    zVal = buffer[5] << 8; //MSB
    zVal = zVal | (buffer[4] & 0x00FF);

    //Invert Direction to match arrows
    zVal = zVal * -1;
    xVal = xVal * -1;
    yVal = yVal * -1;

    return atan2(yVal, zVal) * 57.3;
}
