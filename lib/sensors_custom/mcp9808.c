/**
 ************************************************************************
 * @file mcp9808.c
 * @author Wilfred MK
 * @date 13.05.2021 (Last Updated)
 * @brief mcp9808 temperature driver
 **********************************************************************
 **/
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include "sensors_custom.h"
#include "mcp9808.h"

LOG_MODULE_REGISTER(mcp9808_driver, LOG_LEVEL_DBG);

/**
 * @brief Read/calculate and return the ampbient temperature
 * 
 * @return float 
 */
float mcp9808_read_ambient_temp(void)
{
    const struct device *dev_i2c0 = device_get_binding(DT_LABEL(I2C0));
    uint8_t buffer[2] = {0};
    if (i2c_burst_read(dev_i2c0, MCP_DEV_ADDR, MCP_AMB_TEMP_ADDR, buffer, sizeof(buffer)))
    {
        LOG_ERR("Temperature read error");
    }

    uint8_t upperByte = buffer[0];
    uint8_t lowerByte = buffer[1];

    upperByte = upperByte & 0x1F; //Clear flag bits
    double ambientTemp;

    if ((upperByte & 0x10) == 0x10)
    {
        upperByte = upperByte & 0x0F; //Clear Sign
        ambientTemp = 256 - (upperByte * 16.00 + lowerByte / 16.00);
    }
    else
    {
        ambientTemp = (upperByte * 16.00 + lowerByte / 16.00); //Temp in degrees Cel
    }

    return ambientTemp;
}

/**
 * @brief Powers on the mcp9808 by writing to the
 *  config registers.
 * 
 */
int mcp9808_verify(void)
{
    const struct device *dev_i2c0 = device_get_binding(DT_LABEL(I2C0));
    uint8_t buffer;
    i2c_reg_read_byte(dev_i2c0, MCP_DEV_ADDR, MCP_DEV_ID_ADDR, &buffer);

    if (buffer != MCP_DEV_ID)
    {
        LOG_ERR("Device not verified");
        return -EIO;
    }
    return 0;
}
