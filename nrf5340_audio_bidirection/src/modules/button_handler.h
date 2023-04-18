/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BUTTON_HANDLER_H_
#define _BUTTON_HANDLER_H_

#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>

/** @brief Button ID
 *
 * The pin number of the button
 */
#define BUTTON_ACTIVE_HIGH  true
#define BUTTON_ACTIVE_LOW	false

#define STATE_IDLE	false
#define STATE_DEBOUCING	true
typedef uint32_t button_pin_t;

/** Button actions
 */
enum button_action {
	BUTTON_PRESS,
	BUTTON_LONG_PRESS,
	BUTTON_DOUBLE_PRESS,
	BUTTON_TRIPPLE_PRESS,
	BUTTON_ACTION_NUM,
};

/** Button event
 */
struct button_evt {
	button_pin_t button_pin;
	enum button_action button_action;
};


struct btn_config {
	const char *btn_name;
	uint8_t btn_pin;
	bool button_active;
	bool debounce_state;
	bool current_status;
	bool multi_press_detect;
	uint8_t button_multiple_press_count;
	uint32_t time_between_press;	// detect multiple press
	uint32_t current_tick;
	uint32_t button_timer_tick;
	uint32_t btn_cfg_mask;
};


/*TueTD*/
typedef struct
{
	uint32_t btn_idx;
	uint32_t btn_pin;
}button_t;

/** @brief Initialize button handler, with buttons defined in button_assignments.h.
 *
 * @note This function may only be called once - there is no reinitialize.
 *
 * @return 0 if successful.
 * @return -ENODEV	gpio driver not found
 */
int button_handler_init(void);

/** @brief Check button state.
 *
 * @param[in] button_pin Button pin
 * @param[out] button_pressed Button state. True if currently pressed, false otherwise
 *
 * @return 0 if success, an error code otherwise.
 */
int button_pressed(gpio_pin_t button_pin, bool *button_pressed);

#endif /* _BUTTON_HANDLER_H_ */
