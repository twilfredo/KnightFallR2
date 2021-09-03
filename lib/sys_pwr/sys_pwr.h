#ifndef SYS_PWR_H
#define SYS_PWR_H

#define I2C1 DT_NODELABEL(i2c1)

#define OK 0x00
/* I2C REG ADDRS */
#define BQ24195_ADDR 0x6B
#define INPUT_SRC_CTRL_REG 0x00
#define SYSTEM_STAT_REG 0x08
#define PWR_ON_REG 0x01
#define TIMR_CTRL_REG 0x05
#define MISC_OP_REG 0x07

/* I2C Write Vals*/
#define PWR_SPEC 0x32             /* Sets 500mA Input Current draw, to meet modem power budget */
#define PWR_ON_CONF_CHRG_OFF 0x0B /* Disble the Li-Po Charger */
#define PWR_ON_CONF_CHRG_ON 0x1B
#define TIMR_DISABLE 0x02
#define BAT_FET_DISABLE 0x6B

/* Functions */

int sys_pwr_init(void);

int set_current_limit(void);

int read_power_status(void);

int init_pmic_i2c(void);

int set_charging_off(void);

int set_charging_on(void);

#endif