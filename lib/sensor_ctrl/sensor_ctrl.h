#ifndef SENSOR_CTRL_H
#define SENSOR_CTRL_H
#include <shell/shell.h>

/* Sensor Message Packet */
struct sensor_packet
{
    int16_t turbidity;
    int longitude, lattitude;
};

extern struct k_msgq sensor_msgq;
extern struct k_sem sensor_active_sem;

/* Sensor Defines */
#define SENSOR_PWR_ON true
#define SENSOR_PWR_OFF false

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 500

/* Debug Thread Stack size */
#define STACK_SIZE_SENSOR_CTRL 2048

/* Debug Thread Priority */
#define PRIORITY_SENSOR_CTRL 5 /* Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible  */

/* Function Declarations */
void thread_sensor_control(void);

int turn_sensors_off(void);

int turn_sensors_on(void);

int cmd_sensors_on(const struct shell *shell,
                   size_t argc, char **argv, void *data);

int cmd_sensors_off(const struct shell *shell,
                    size_t argc, char **argv, void *data);

int cmd_sensors_single_read(const struct shell *shell,
                            size_t argc, char **argv, void *data);
#endif