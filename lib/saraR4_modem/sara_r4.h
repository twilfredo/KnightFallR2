
/**
 ************************************************************************
 * @file sara_r4.h
 * @author Wilfred MK
 * @date 13.05.2021 (Last Updated)
 * @brief Custom sara-r4 communication module.
 **********************************************************************
 **/
#ifndef SARA_R4_C
#define SARA_R4_C

#include "sensor_ctrl.h"

/* ==================================================================== */
/* ==============================THREAD DEFINES======================== */
/* ==================================================================== */
/* Debug Thread Stack size */
#define STACK_SIZE_MODEM_THREAD 4096

/* Debug Thread Priority */
#define THREAD_PRIORITY_MODEM 1 /* Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible  */

/* ==================================================================== */
/* ===============================GLOBALS============================== */
/* ==================================================================== */
extern bool mqttConnected;
extern struct k_msgq to_network_msgq;
extern struct k_sem networkReady;

/* ==================================================================== */
/* ==============================MODEM PINS============================ */
/* ==================================================================== */
/* Device tree node identifier for GPIO0 */
#define GPIO0 DT_LABEL(DT_NODELABEL(gpio0))
#define SARA_PWR_PIN 0x10
#define SARA_RST_PIN 0xc
#define SARA_VINT_PIN 0x2

#define MDM_POWER_ENABLE 1
#define MDM_POWER_DISABLE 0
#define MDM_RESET_NOT_ASSERTED 1
#define MDM_RESET_ASSERTED 0

/* ==================================================================== */
/* ==============================MISC Defines========================== */
/* ==================================================================== */
#define MAX_READ_SIZE 128

/* ==================================================================== */
/* =================MODEM POWER CONTROL DEFINES======================== */
/* ==================================================================== */

/* pin used to enable the buffer power */
#define SERIAL_BUFFER_ENABLE_GPIO_NAME DT_LABEL(DT_INST(0, nordic_nrf_gpio))
#define SERIAL_BUFFER_ENABLE_GPIO_PIN 25
#define SERIAL_BUFFER_ENABLE_GPIO_FLAGS GPIO_ACTIVE_LOW

/* pin used to detect V_INT (buffer power) */
#define V_INT_DETECT_GPIO_PIN 2
#define V_INT_DETECT_GPIO_FLAGS GPIO_ACTIVE_HIGH

/* SKYWORKS SKY13351 antenna selection settings (only use vctl1) */
#define ANT_UFLn_GPIO_NAME DT_GPIO_LABEL(DT_INST(0, skyworks_sky13351), vctl1_gpios)
#define ANT_UFLn_GPIO_FLAGS DT_GPIO_FLAGS(DT_INST(0, skyworks_sky13351), vctl1_gpios)
#define ANT_UFLn_GPIO_PIN DT_GPIO_PIN(DT_INST(0, skyworks_sky13351), vctl1_gpios)

/* Sensor Data */
#define SD_LEN 12

/* Modem loop switch variables for updating MQTT fields */
#define TBD_FIELD 0x01
#define LONG_FIELD 0x6A
#define LATT_FIELD 0x2A
#define NUM_PACKET_TO_SEND 3
#define THINGSPEAK_UPDATE_RATE 15 //Seconds

#define GPS_NO_LOCK_VAL 0.00 //Constant to indicate NOLOCK
/* ==================================================================== */
/* ==============================UART-PORT1============================ */
/* ==================================================================== */
/* Device tree node identifier for UART1 */
#define UART1 DT_LABEL(DT_NODELABEL(uart1))

int modem_config_pins(void);

int modem_uart_init(void);

void thread_modem_ctrl(void *p1, void *p2, void *p3);

void thread_modem_receive(void *p1, void *p2, void *p3);

int modem_pin_init(void);

void modem_uart_tx(char *command);

bool modem_recv(void);

void mdm_receiver_flush(const struct device *uart_device);

bool modem_network_init(void);

bool modem_mqtt_init(void);

void update_sensor_buffers(struct sensor_packet *sensorData);

#endif