#ifndef SAM_M8Q_H
#define SAM_M8Q_H

/* ==================================================================== */
/* ==============================THREAD DEFINES======================== */
/* ==================================================================== */
/* Debug Thread Stack size */
#define STACK_SIZE_GPS_THREAD 2048

/* ==================================================================== */
/* ==============================MISC Defines========================== */
/* ==================================================================== */
#define MAX_READ_SIZE 128

/* Debug Thread Priority */
/* Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible  */
#define THREAD_PRIORITY_GPS 3

/* Device tree node identifier for UART0 - Connected to GPS Module*/
#define UART0 DT_LABEL(DT_NODELABEL(uart0))

void thread_gps_ctrl(void);

void thread_gps_receive(void);

int sam_m8q_uart_init(void);

void sam_m8q_uart_tx(char *command);

void sam_mdm_receiver_flush(const struct device *uart_device);

bool sam_recv(void);
#endif