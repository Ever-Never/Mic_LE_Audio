/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 * BYTECH-JSC
 * TUETD modify":D
 */

#include "led.h"

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led, 5);

#define BLINK_FREQ_MS 1000
/* Maximum number of LED_UNITS. 1 RGB LED = 1 UNIT of 3 LEDS */
#define LED_UNIT_MAX 2
#define NUM_COLORS_RGB 3
#define BASE_10 10
#define DT_LABEL_AND_COMMA(node_id) DT_PROP(node_id, label),
#define GPIO_DT_SPEC_GET_AND_COMMA(node_id) GPIO_DT_SPEC_GET(node_id, gpios),

/* The following arrays are populated compile time from the .dts*/
static const char *const led_labels[] = {DT_FOREACH_CHILD(DT_PATH(user_leds), DT_LABEL_AND_COMMA) };

static const struct gpio_dt_spec leds[] = { DT_FOREACH_CHILD(DT_PATH(user_leds),
							     GPIO_DT_SPEC_GET_AND_COMMA) };

enum led_type {
	LED_MONOCHROME,
	LED_COLOR,
};

struct user_config {
	bool blink;
	bool current_status;
	/*Control blink speed*/
	uint8_t blink_duration; 
	uint8_t blink_current_tick;
	uint32_t blink_times;
	uint32_t on_length;
	enum led_color color;
};

struct led_unit_cfg {
	uint8_t led_no;
	enum led_type unit_type;
	union {
		const struct gpio_dt_spec *mono;
		const struct gpio_dt_spec *color[NUM_COLORS_RGB];
	} type;
	struct user_config user_cfg;
};

static uint8_t leds_num;
static bool initialized;
static struct led_unit_cfg led_units[LED_UNIT_MAX];
static bool m_last_time_stop_stream = false;
/**
 * @brief Configures fields for a RGB LED
 */
static int configure_led_color(uint8_t led_unit, uint8_t led_color, uint8_t led)
{
	if (!device_is_ready(leds[led].port)) {
		LOG_ERR("LED GPIO controller not ready");
		return -ENODEV;
	}

	led_units[led_unit].type.color[led_color] = &leds[led];
	led_units[led_unit].unit_type = LED_COLOR;

	return gpio_pin_configure_dt(led_units[led_unit].type.color[led_color],
				     GPIO_OUTPUT_INACTIVE);
}

/**
 * @brief Configures fields for a monochrome LED
 */
static int config_led_monochrome(uint8_t led_unit, uint8_t led)
{
	if (!device_is_ready(leds[led].port)) {
		LOG_ERR("LED GPIO controller not ready");
		return -ENODEV;
	}

	led_units[led_unit].type.mono = &leds[led];
	led_units[led_unit].unit_type = LED_MONOCHROME;
	led_units[led_unit].user_cfg.blink_duration = 1;
	led_units[led_unit].user_cfg.blink_current_tick = 0;
	led_units[led_unit].user_cfg.current_status = false;
	return gpio_pin_configure_dt(led_units[led_unit].type.mono, GPIO_OUTPUT_INACTIVE);
}

/**
 * @brief Parses the device tree for LED settings.
 */
static int led_device_tree_parse(void)
{
	/*Config all leds as monochrome:D*/
	int ret;
	for (uint8_t i = 0; i < leds_num; i++) {
		ret = config_led_monochrome(i, i);
		LOG_INF("Config led: %s", led_labels[i]);
		if (ret) {			
			return ret;
		}
	}
	return 0;
}

/**
 * @brief Internal handling to set the status of a led unit
 */
static int led_set_int(uint8_t led_unit, enum led_color color)
{
	int ret;

	if (led_units[led_unit].unit_type == LED_MONOCHROME) {
		if (color) {
			ret = gpio_pin_set_dt(led_units[led_unit].type.mono, 1);
			if (ret) {
				return ret;
			}
		} else {
			ret = gpio_pin_set_dt(led_units[led_unit].type.mono, 0);
			if (ret) {
				return ret;
			}
		}
	}
	return 0;
}

static void led_blink_work_handler(struct k_work *work);

K_WORK_DEFINE(led_blink_work, led_blink_work_handler);

/**
 * @brief Submit a k_work on timer expiry.
 */
void led_blink_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&led_blink_work);
}

K_TIMER_DEFINE(led_blink_timer, led_blink_timer_handler, NULL);

/**
 * @brief Periodically invoked by the timer to blink LEDs.
 */
static void led_blink_work_handler(struct k_work *work)
{
	int ret;
	//static bool on_phase;

	for (uint8_t i = 0; i < leds_num; i++) {
		if (led_units[i].user_cfg.blink) {
			if (led_units[i].user_cfg.current_status == false) {
				if(led_units[i].user_cfg.blink_current_tick++ >= led_units[i].user_cfg.blink_duration)
				{
					if(led_units[i].user_cfg.blink_times)
					{
						if(led_units[i].user_cfg.blink_times != LED_BLINK_FOREVER)
						{
							led_units[i].user_cfg.blink_times--;
						}
						led_units[i].user_cfg.blink_current_tick = 0;
						led_units[i].user_cfg.current_status = true; 
						ret = led_set_int(i, LED_COLOR_RED);
						ERR_CHK(ret);
					}
				}
			}else {
				if(led_units[i].user_cfg.blink_current_tick++ >= led_units[i].user_cfg.blink_duration)
				{
					led_units[i].user_cfg.blink_current_tick = 0;
					led_units[i].user_cfg.current_status = false; 
					ret = led_set_int(i, LED_COLOR_OFF);
					ERR_CHK(ret);
				}
			}
		}
		else 
		{
			if(led_units[i].user_cfg.on_length == LED_ON_FOREVER)
			{
				gpio_pin_set_dt(led_units[i].type.mono, 1);
			}
			else
			{
				if(led_units[i].user_cfg.on_length)
				{
					led_units[i].user_cfg.on_length--;
					gpio_pin_set_dt(led_units[i].type.mono, 1);
				}
				else
				{
					gpio_pin_set_dt(led_units[i].type.mono, 0);
				}
			}
		}
	}

	//on_phase = !on_phase;
}

static int led_set(uint8_t led_unit, enum led_color color, uint32_t on_times, uint32_t blink_times, bool blink)
{
	int ret;

	if (!initialized) {
		return -EPERM;
	}

	ret = led_set_int(led_unit, color);
	if (ret) {
		return ret;
	}

	led_units[led_unit].user_cfg.blink = blink;
	led_units[led_unit].user_cfg.color = color;
	if(blink)
	{
		led_units[led_unit].user_cfg.blink_times = blink_times;
		led_units[led_unit].user_cfg.blink_duration = on_times;
		led_units[led_unit].user_cfg.on_length = 0;
	}
	else
	{
		led_units[led_unit].user_cfg.on_length = on_times * 2;
		led_units[led_unit].user_cfg.blink_times = 0; 
	}

	return 0;
}

int led_on(uint8_t led_unit, uint32_t turn_on_time, ...)
{
	if (led_units[led_unit].unit_type == LED_MONOCHROME) {
		return led_set(led_unit, LED_ON, turn_on_time, 0,LED_SOLID);
	}
	else
	{
		LOG_ERR("Failed to set LED\r\n");
		return -1;
	}
}

int led_blink(uint8_t led_unit, uint8_t on_off_duration, uint32_t blink_times)
{
	if (led_units[led_unit].unit_type == LED_MONOCHROME) {
		return led_set(led_unit, LED_ON, on_off_duration, blink_times, LED_BLINK);
	}
	else
	{
		LOG_ERR("Failed to set led blink\r\n");
		return -1;
	}
}

int led_off(uint8_t led_unit)
{
	return led_set(led_unit, LED_COLOR_OFF, 0, 0, LED_SOLID);
}

int led_init(void)
{
	int ret;

	if (initialized) {
		return -EPERM;
	}

	__ASSERT(ARRAY_SIZE(leds) != 0, "No LEDs found in dts");

	leds_num = ARRAY_SIZE(leds);

	ret = led_device_tree_parse();
	if (ret) {
		return ret;
	}

	k_timer_start(&led_blink_timer, K_MSEC(BLINK_FREQ_MS / 20), K_MSEC(BLINK_FREQ_MS / 20));
	initialized = true;
	return 0;
}
bool app_led_is_on(uint8_t led_unit)
{
	if(led_units[led_unit].user_cfg.on_length ||  led_units[led_unit].user_cfg.blink_times)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void app_led_indicator_stop_both()
{
	led_off(LED_POWER);
	led_off(LED_BLE);
}
/**/
void app_led_indicator_in_pair()
{
	led_blink(LED_POWER, 16, LED_BLINK_FOREVER);
}
void app_led_indicator_pair_success()
{
	led_blink(LED_POWER, 4, 10);
	m_last_time_stop_stream = false;
}
void app_led_indicator_pair_timeout()
{
	led_off(LED_POWER);
	m_last_time_stop_stream = false;

}
/**/

/*Both led blink normal -> streaming*/
void app_led_indicator_streaming()
{
	led_blink(LED_BLE, 4 , LED_BLINK_FOREVER);
	//m_last_time_stop_stream = true;
	if(m_last_time_stop_stream == true)
	{
		led_off(LED_POWER);
		m_last_time_stop_stream = false;
	}
	//led_off(LED_POWER);
}
void app_led_indicator_stop_streaming()
{
	led_on(LED_POWER, LED_ON_FOREVER);	
	led_off(LED_BLE);
	m_last_time_stop_stream = true;
}
/*Indicate device is powered*/
void app_led_turn_on_device()
{
	led_on(LED_POWER, LED_ON_FOREVER);
}