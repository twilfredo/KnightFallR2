#ifndef SAM_M8Q_H
#define SAM_M8Q_H

/* Device tree node identifier for UART0 - Connected to GPS Module*/
#define UART0 DT_LABEL(DT_NODELABEL(uart0))

int sam_m8q_uart_init(void);

void sam_m8q_uart_tx(char *command);

#endif