#include <drivers/sensor.h>
#include <logging/log.h>
#include "mcp9808.h"

#define MCP9808_DEVNAME DT_LABEL(DT_INST(0, microchip_mcp9808))

LOG_MODULE_REGISTER(mcp9808_abstract_driver, LOG_LEVEL_DBG);

/**
 * Reads ambient temperature from MCP9808 
 * Helper function to abstract out sensor API
 */
double read_ambient_temperature(void)
{
    const struct device *dev = device_get_binding(MCP9808_DEVNAME);

    if (dev == NULL)
    {
        LOG_ERR("Device struct could not be found");
        return -EINVAL;
    }

    struct sensor_value sVal = {};
    if (sensor_sample_fetch(dev) < 0 || sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &sVal) < 0)
    {
        LOG_ERR("Error fetching sensor data");
        return -EINVAL;
    }

    return sensor_value_to_double(&sVal);
}