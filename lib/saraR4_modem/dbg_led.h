#ifndef DBG_LED_H
#define DBG_LED_H

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS_FAST 500
#define SLEEP_TIME_MS_SLOW 1000

/* Debug Thread Stack size */
#define STACK_SIZE_LED_THREAD 128

/* Debug Thread Priority */
#define THREAD_PRIORITY_LED_THREAD 10 /* Lower Numerics has higher priority, -Ve Priorities are cooperitive threads, +Ve Priorities  are Preemtible  */

/* The devicetree node identifier for the "led0" alias. */
#define BLUE_LED_NODE DT_ALIAS(led3)
#define RED_LED_NODE DT_ALIAS(led1)

#define BLUE_LED DT_GPIO_LABEL(BLUE_LED_NODE, gpios)
#define RED_LED DT_GPIO_LABEL(RED_LED_NODE, gpios)

#define BLUE_PIN DT_GPIO_PIN(BLUE_LED_NODE, gpios)
#define RED_PIN DT_GPIO_PIN(RED_LED_NODE, gpios)

#define BLUE_LED_FLAGS DT_GPIO_FLAGS(BLUE_LED_NODE, gpios)
#define RED_LED_FLAGS DT_GPIO_FLAGS(RED_LED_NODE, gpios)
// Thread to flash a debug LED
void thread_flash_debug_led(void);

#endif