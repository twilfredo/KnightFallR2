#ifndef ADXL343_H
#define ADXL343_H

#define ADXL_DEV_ADDR 0x53 //I2C DEV ADDR
#define ADXL_DEVID_ADDR 0x00
#define ADXL_POWER_CTL_ADDR 0x2D
#define ADXL_WHOAMI_VAL 0xE5
#define ADXL_MEASURE_4HZ 0x09
#define ADXL_READ_START_ADDR 0x32
#define ADXL_DATA_FMT_ADDR 0x31
//Full Res, 4G
#define ADXL_DATA_RANGE_4G 0x09

int adxl343_verify(void);

void adxl343_set_measurement_mode(void);

void adxl343_read_xyz(int16_t *bufferSave);

double adxl343_get_roll(void);

double adxl343_get_pitch(void);

#endif