/**
 ************************************************************************
 * @file sys_pwr.c
 * @author Wilfred MK
 * @date 28.08.2021 (Last Updated)
 * @brief Control the PMIC (BQ24195) onboard the particle boron, and is used to 
 *          set the power requirements as per project power budget. 
 **********************************************************************
 **/
#include <drivers/i2c.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "sys_pwr.h"
#include "dbg_led.h"

LOG_MODULE_REGISTER(sys_pwr, LOG_LEVEL_DBG);

int sys_pwr_init(void)
{
    init_usr_led();
    turn_usr_led_on();
    if (init_pmic_i2c() != OK)
    {
        LOG_ERR("Unable config PMIC I2C");
    }

    if (set_current_limit() != OK)
    {
        LOG_ERR("Unable to set current limits");
        return -1;
    }

    if (read_power_status() != OK)
    {
        LOG_ERR("Unstable Power");
        return -1;
    }

    if (set_charging_off() != OK)
    {
        LOG_ERR("Unable to turn charging off");
    }
    //CHECK POWER REG STAT
    LOG_INF("System Power OK...");

    turn_usr_led_off();
    k_msleep(200);
    turn_usr_led_on();
    k_msleep(50);
    turn_usr_led_off();
    return 0;
}

int set_charging_off(void)
{
    uint8_t val;
    const struct device *dev_i2c1 = device_get_binding(DT_LABEL(I2C1));

    /* Set charging off */
    if (i2c_reg_write_byte(dev_i2c1, BQ24195_ADDR, MISC_OP_REG, BAT_FET_DISABLE) != OK)
    {
        LOG_DBG("Unable to misc operation control register");
        return -1;
    }

    if (i2c_reg_read_byte(dev_i2c1, BQ24195_ADDR, MISC_OP_REG, &val) != OK)
    {
        LOG_DBG("Unable to read to System Status Register register");
        return -1;
    }

    if (val != BAT_FET_DISABLE)
    {
        LOG_DBG("BAT_FET_DISABLE write check mismatch");
        return -1;
    }

    if (i2c_reg_write_byte(dev_i2c1, BQ24195_ADDR, TIMR_CTRL_REG, TIMR_DISABLE) != OK)
    {
        LOG_DBG("Unable to set timer control register");
        return -1;
    }

    if (i2c_reg_read_byte(dev_i2c1, BQ24195_ADDR, TIMR_CTRL_REG, &val) != OK)
    {
        LOG_DBG("Unable to read to System Status Register register");
        return -1;
    }

    if (val != TIMR_DISABLE)
    {
        LOG_DBG("TIMR_DISABLE write check mismatch");
        return -1;
    }

    if (i2c_reg_write_byte(dev_i2c1, BQ24195_ADDR, PWR_ON_REG, PWR_ON_CONF_CHRG_OFF) != OK)
    {
        LOG_DBG("Unable to set source control register");
        return -1;
    }

    if (i2c_reg_read_byte(dev_i2c1, BQ24195_ADDR, PWR_ON_REG, &val) != OK)
    {
        LOG_DBG("Unable to read to System Status Register register");
        return -1;
    }

    if (val != PWR_ON_CONF_CHRG_OFF)
    {
        LOG_DBG("CHRG_OFF check mismatch 3 %d", val);
        return -1;
    }

    return 0;
}

int set_charging_on(void)
{
    uint8_t val;
    const struct device *dev_i2c1 = device_get_binding(DT_LABEL(I2C1));

    /* Set charging off */
    if (i2c_reg_write_byte(dev_i2c1, BQ24195_ADDR, PWR_ON_REG, PWR_ON_CONF_CHRG_ON) != OK)
    {
        LOG_DBG("Unable to set source control register");
        return -1;
    }

    if (i2c_reg_read_byte(dev_i2c1, BQ24195_ADDR, PWR_ON_REG, &val) != OK)
    {
        LOG_DBG("Unable to read to System Status Register register");
        return -1;
    }

    if (val != PWR_ON_CONF_CHRG_ON)
    {
        LOG_DBG("Write check mismatch");
        return -1;
    }

    return 0;
}

int read_power_status(void)
{
    uint8_t val;
    const struct device *dev_i2c1 = device_get_binding(DT_LABEL(I2C1));

    if (i2c_reg_read_byte(dev_i2c1, BQ24195_ADDR, SYSTEM_STAT_REG, &val) != OK)
    {
        LOG_DBG("Unable to read to System Status Register register");
        return -1;
    }

    uint8_t pwrGood = 0x04;
    val &= pwrGood;

    uint8_t pwrGoodBit = val >> 2;

    if (pwrGoodBit != 1)
    {
        LOG_WRN("PG_STAT, Power not good");
        return -1;
    }

    return 0;
}

int init_pmic_i2c(void)
{
    const struct device *dev_i2c1 = device_get_binding(DT_LABEL(I2C1));

    if (i2c_configure(dev_i2c1, I2C_SPEED_FAST | I2C_MODE_MASTER) != OK)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief Sets the input current limit of the IC to be 500mA
 *          allows for the IC to draw 500mA from the battery.
 */
int set_current_limit(void)
{
    const struct device *dev_i2c1 = device_get_binding(DT_LABEL(I2C1));
    uint8_t val = 0;
    /* Set the limit to 500mA */
    if (i2c_reg_write_byte(dev_i2c1, BQ24195_ADDR, INPUT_SRC_CTRL_REG, PWR_SPEC) != OK)
    {
        LOG_DBG("Unable to set source control register");
        return -1;
    }

    k_msleep(200);

    if (i2c_reg_read_byte(dev_i2c1, BQ24195_ADDR, INPUT_SRC_CTRL_REG, &val) != OK)
    {
        LOG_DBG("Unable to read source control register");
        return -1;
    }

    if (val != PWR_SPEC)
    {
        LOG_DBG("Unable to verify source control register");
        return -1;
    }

    return 0;
}