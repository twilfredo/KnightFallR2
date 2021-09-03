#ifndef SENSOR_CTRL_H
#define SENSOR_CTRL_H
#include <shell/shell.h>

/* Sensor Message Packet */
struct sensor_packet
{
    int turbidity; //Stored in NTUs
    float longitude, lattitude;
};

extern struct k_msgq sensor_msgq;
extern struct k_sem sensor_active_sem, tsd10_read_sem, gps_read_sem;
extern struct k_poll_signal tsd10_sig;

/* Sensor Defines */
#define SENSOR_PWR_ON true
#define SENSOR_PWR_OFF false
#define VRAIL_DELAY 100
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 500
#define TSD_EVENT_TIMEOUT 1000

/* Debug Thread Stack size */
#define STACK_SIZE_SENSOR_CTRL 2048

/* Debug Thread Priority */
#define PRIORITY_SENSOR_CTRL 5 /* Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible  */

/* Function Declarations */
void thread_sensor_control(void *p1, void *p2, void *p3);

int turn_sensors_off(void);

int turn_sensors_on(void);

int cmd_sensors_on(const struct shell *shell,
                   size_t argc, char **argv, void *data);

int cmd_sensors_off(const struct shell *shell,
                    size_t argc, char **argv, void *data);

int cmd_tsd_off(const struct shell *shell,
                size_t argc, char **argv, void *data);

int cmd_tsd_on(const struct shell *shell,
               size_t argc, char **argv, void *data);

int cmd_gps_off(const struct shell *shell,
                size_t argc, char **argv, void *data);

int cmd_gps_on(const struct shell *shell,
               size_t argc, char **argv, void *data);

int cmd_sensors_single_read(const struct shell *shell,
                            size_t argc, char **argv, void *data);

void get_turbidity(struct sensor_packet *sensorData);

void get_gps(struct sensor_packet *sensorData);

#endif