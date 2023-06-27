/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>
#include <stdbool.h>

#define LED_APP_RGB 0
#define LED_NET_RGB 1
#define LED_APP_1_BLUE 2
#define LED_APP_2_GREEN 3
#define LED_APP_3_GREEN 4

#define LED_ON_FOREVER	UINT32_MAX
#define LED_BLINK_FOREVER	UINT32_MAX



#define RED 0
#define GREEN 1
#define BLUE 2

#define GRN GREEN
#define BLU BLUE

enum led_color {
	LED_COLOR_OFF, /* 000 */
	LED_COLOR_RED, /* 001 */
	LED_COLOR_GREEN, /* 010 */
	LED_COLOR_YELLOW, /* 011 */
	LED_COLOR_BLUE, /* 100 */
	LED_COLOR_MAGENTA, /* 101 */
	LED_COLOR_CYAN, /* 110 */
	LED_COLOR_WHITE, /* 111 */
	LED_COLOR_NUM,
};



/*user define LED name*/
#define LED_POWER 0	
#define LED_BLE	1
#define LED_ON LED_COLOR_WHITE

#define LED_BLINK true
#define LED_SOLID false

/**
 * @brief Set the state of a given LED unit to blink.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @note		If the given LED unit is an RGB LED, color must be
 *			provided as a single vararg. See led_color.
 *			For monochrome LEDs, the vararg will be ignored.
 *			Using a LED unit assigned to another core will do nothing and return 0.
 * @return		0 on success
 *			-EPERM if the module has not been initialised
 *			-EINVAL if the color argument is illegal
 *			Other errors from underlying drivers.
 */
int led_blink(uint8_t led_unit, uint8_t on_off_duration, uint32_t blink_times);

/**
 * @brief Turn the given LED unit on.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @note		If the given LED unit is an RGB LED, color must be
 *			provided as a single vararg. See led_color.
 *			For monochrome LEDs, the vararg will be ignored.
*			Using a LED unit assigned to another core will do nothing and return 0.
 * @return		0 on success
 *			-EPERM if the module has not been initialised
 *			-EINVAL if the color argument is illegal
 *			Other errors from underlying drivers.
 */
int led_on(uint8_t led_unit, uint32_t turn_on_time, ...);

/**
 * @brief Set the state of a given LED unit to off.
 *
 * @note A led unit is defined as an RGB LED or a monochrome LED.
 *		Using a LED unit assigned to another core will do nothing and return 0.
 *
 * @param led_unit	Selected LED unit. Defines are located in board.h
 * @return		0 on success
 *			-EPERM if the module has not been initialised
 *			-EINVAL if the color argument is illegal
 *			Other errors from underlying drivers.
 */
int led_off(uint8_t led_unit);

/**
 * @brief Initialise the LED module
 *
 * @note This will parse the .dts files and configure all LEDs.
 *
 * @return	0 on success
 *		-EPERM if already initialsed
 *		-ENXIO if a LED is missing unit number in dts
 *		-ENODEV if a LED is missing color identifier
 */
int led_init(void);


bool app_led_is_on(uint8_t led_unit);


void app_led_turn_on_device(void);

void app_led_indicator_pair_timeout(void);

void app_led_indicator_stop_streaming(void);

void app_led_indicator_streaming(void);

void app_led_indicator_in_pair(void);

void app_led_indicator_stop_both(void);

void app_led_indicator_pair_success(void);

#endif /* _LED_H_ */
