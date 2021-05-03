/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include "main.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 500

LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);

/* Compile Time Thread Init */
K_THREAD_DEFINE(debug_led, STACK_SIZE_LED_THREAD, thread_flash_debug_led, NULL, NULL, NULL, THREAD_PRIORITY_LED_THREAD, 0, 50);

void main(void)
{
}

/**
 * @brief This thread indicates network conenction status.
 * 		Status 1: Fast Blink when not connected.
 * 		Status 2: Slow Blink when connected.
 * 
 */
void thread_flash_debug_led(void)
{

	bool led_is_on = true;

	const struct device *dev = device_get_binding(BLUE_LED);
	const struct device *dev1 = device_get_binding(RED_LED);

	int ret = gpio_pin_configure(dev, BLUE_PIN, GPIO_OUTPUT_ACTIVE | BLUE_LED_FLAGS);
	int ret1 = gpio_pin_configure(dev1, RED_PIN, GPIO_OUTPUT_ACTIVE | RED_LED_FLAGS);

	if (ret < 0 || ret1 < 0)
	{
		LOG_ERR("LED GPIO ERROR");
		return;
	}

	while (1)
	{
		gpio_pin_set(dev, BLUE_PIN, (int)led_is_on);
		gpio_pin_set(dev, RED_PIN, (int)led_is_on);

		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
	}
}
