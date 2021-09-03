#ifndef DBG_LED_H
#define DBG_LED_H

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS_FAST 5000
#define SLEEP_TIME_MS_SLOW 5000

/* Debug Thread Stack size */
#define STACK_SIZE_LED_THREAD 128

/* Debug Thread Priority */
#define THREAD_PRIORITY_LED_THREAD 10 /* Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible  */

/* The devicetree node identifier for the "led0" alias. */
#define BLUE_LED_NODE DT_ALIAS(led3)
#define RED_LED_NODE DT_ALIAS(led1)
#define USR_LED_NODE DT_ALIAS(led0)
#define GRN_LED_NODE DT_ALIAS(led2)

#define BLUE_LED DT_GPIO_LABEL(BLUE_LED_NODE, gpios)
#define RED_LED DT_GPIO_LABEL(RED_LED_NODE, gpios)
#define USR_LED DT_GPIO_LABEL(USR_LED_NODE, gpios)
#define GRN_LED DT_GPIO_LABEL(GRN_LED_NODE, gpios)

#define BLUE_PIN DT_GPIO_PIN(BLUE_LED_NODE, gpios)
#define RED_PIN DT_GPIO_PIN(RED_LED_NODE, gpios)
#define USR_PIN DT_GPIO_PIN(USR_LED_NODE, gpios)
#define GRN_PIN DT_GPIO_PIN(GRN_LED_NODE, gpios)

#define BLUE_LED_FLAGS DT_GPIO_FLAGS(BLUE_LED_NODE, gpios)
#define RED_LED_FLAGS DT_GPIO_FLAGS(RED_LED_NODE, gpios)
#define USR_LED_FLAGS DT_GPIO_FLAGS(USR_LED_NODE, gpios)
#define GRN_LED_FLAGS DT_GPIO_FLAGS(GRN_LED_NODE, gpios)

// Thread to flash a debug LED
void thread_flash_debug_led(void *p1, void *p2, void *p3);

int init_usr_led(void);

int turn_usr_led_on(void);

int turn_usr_led_off(void);

#endif