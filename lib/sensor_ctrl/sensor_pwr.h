/**
 ************************************************************************
 * @file sensor_pwr.h
 * @author Wilfred MK
 * @date 23.06.2021 (Last Updated)
 * @brief Driver control the GPIO pins that drives Vgs to the mosfets 
 *          power control the sensors (GPS, Turbidity)
 **********************************************************************
 **/
#ifndef SENSOR_PWR_H
#define SENSOR_PWR_H

#define GPIO_FET DT_LABEL(DT_NODELABEL(gpio0))
#define SAM_M8Q_PWR_PIN 4 //A1 - P0.04
#define TSD_10_PWR_PIN 28 //A2 - P0.28
#define PWR_ON 1
#define PWR_OFF 0

extern bool sensorPwrState;

int init_sensor_pwr_gpio(void);

int sam_m8q_pwr_on(void);

int tsd_10_pwr_on(void);

int sam_m8q_pwr_off(void);

int tsd_10_pwr_off(void);

bool get_gps_pwr_stat(void);

#endif