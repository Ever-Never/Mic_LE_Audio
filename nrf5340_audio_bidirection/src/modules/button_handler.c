/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "button_handler.h"
#include "button_assignments.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "macros_common.h"
#include "ctrl_events.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(button_handler, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);
#define BUTTON_DEBOUNCE		50
#define BUTTON_LONGPRESS	3000
/*total tick to be refered as two time pressed*/
#define BUTTON_TIME_BETWEEN_PRESS	1000	


/* How many buttons does the module support. Increase at memory cost */
#define BUTTONS_MAX 4
#define BASE_10 10

static bool debounce_is_ongoing;
static struct gpio_callback btn_callback[BUTTONS_MAX];
static const struct device *gpio_53_dev;
static const struct device *gpio_54_dev;
static const struct gpio_dt_spec *button_spec;


static void button_scan(uint8_t button_idx);

static struct btn_config btn_cfg[] = {
	{
		.btn_name = STRINGIFY(BUTTON_VOLUME_DOWN),
		.btn_pin = BUTTON_VOLUME_DOWN,
		.btn_cfg_mask = DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios),
		.button_active = BUTTON_ACTIVE_LOW,
	},
	{
		.btn_name = STRINGIFY(BUTTON_VOLUME_UP),
		.btn_pin = BUTTON_VOLUME_UP,
		.btn_cfg_mask = DT_GPIO_FLAGS(DT_ALIAS(sw1), gpios),
		.button_active = BUTTON_ACTIVE_LOW,
	},
	{
		.btn_name = STRINGIFY(BUTTON_PAIR),
		.btn_pin = BUTTON_PAIR,
		.btn_cfg_mask = DT_GPIO_FLAGS(DT_ALIAS(sw2), gpios),
		.button_active = BUTTON_ACTIVE_LOW,
	},
	{
		.btn_name = STRINGIFY(BUTTON_ON_OFF),
		.btn_pin = BUTTON_ON_OFF,
		.btn_cfg_mask = DT_GPIO_FLAGS(DT_ALIAS(sw3), gpios),
		.button_active = BUTTON_ACTIVE_HIGH,
	},
};

typedef void (*button_send_event)(uint8_t button_index);
static void button_send_event_pressed(uint8_t button_index)
{
	int ret = 0;
	struct event_t event;
	LOG_INF("Button %d pressed\r\n", button_index);
	event.button_activity.button_pin = btn_cfg[button_index].btn_pin;
	event.button_activity.button_action = BUTTON_PRESS;
	event.event_source = EVT_SRC_BUTTON;
	if (ctrl_events_queue_empty()) {
		ret = ctrl_events_put(&event);
		ERR_CHK(ret);
	} else {
		LOG_WRN("Event queue is not empty, try again later");
	}
}
static void button_send_event_double_pressed(uint8_t button_index)
{
	LOG_INF("Button %d double pressed\r\n", button_index);
	int ret = 0;
	struct event_t event;
	event.button_activity.button_pin = btn_cfg[button_index].btn_pin;
	event.button_activity.button_action = BUTTON_DOUBLE_PRESS;
	event.event_source = EVT_SRC_BUTTON;
	if (ctrl_events_queue_empty()) {
		ret = ctrl_events_put(&event);
		ERR_CHK(ret);
	} else {
		LOG_WRN("Event queue is not empty, try again later");
	}
}
static void button_send_event_tripple_pressed(uint8_t button_index)
{
	int ret = 0;
	LOG_INF("Button %d tripple pressed\r\n", button_index);
	struct event_t event;
	event.button_activity.button_pin = btn_cfg[button_index].btn_pin;
	event.button_activity.button_action = BUTTON_TRIPPLE_PRESS;
	event.event_source = EVT_SRC_BUTTON;
	if (ctrl_events_queue_empty()) {
		ret = ctrl_events_put(&event);
		ERR_CHK(ret);
	} else {
		LOG_WRN("Event queue is not empty, try again later");
	}
}
const static button_send_event button_press_send_event[] =
{
	button_send_event_pressed,
	button_send_event_double_pressed,
	button_send_event_tripple_pressed
};

static void button_0_timer(struct k_timer *timer)
{
	button_scan(0);

}
static void button_1_timer(struct k_timer *timer)
{
	button_scan(1);
}

static void button_2_timer(struct k_timer *timer)
{
	button_scan(2);
}

static void button_3_timer(struct k_timer *timer)
{
	button_scan(3);
}

K_TIMER_DEFINE(button0_timer, button_0_timer, NULL);
K_TIMER_DEFINE(button1_timer, button_1_timer, NULL);
K_TIMER_DEFINE(button2_timer, button_2_timer, NULL);
K_TIMER_DEFINE(button3_timer, button_3_timer, NULL);

static struct k_timer *buttons_timer[4] = {
	&button0_timer,
	&button1_timer,
	&button2_timer,
	&button3_timer
};
/**@brief Simple debouncer for buttons
 *
 * @note Needed as low-level driver debouce is not
 * implemented in Zephyr for nRF53 yet
 */

static void button_scan(uint8_t button_idx)
{
	struct device *p_gpio = gpio_53_dev;
	if(button_idx == 1)
	{
		p_gpio = gpio_54_dev;
	}
	btn_cfg[button_idx].current_status = gpio_pin_get(p_gpio, btn_cfg[button_idx].btn_pin);
	if(btn_cfg[button_idx].button_active == BUTTON_ACTIVE_HIGH) // revert logic if active high
	{
		btn_cfg[button_idx].current_status = !btn_cfg[button_idx].current_status;
	}
	/*neu doc ve la LOW => button is pressing => continue counting*/
	/*if read high => button release -> calulate pressed time and send event queue*/
	if(btn_cfg[button_idx].current_status != 0)
	{
		btn_cfg[button_idx].current_tick++;
		LOG_DBG("Button :%d - tick:%u\r\n", button_idx, btn_cfg[button_idx].current_tick);
		if(btn_cfg[button_idx].current_tick > BUTTON_LONGPRESS)
		{
			btn_cfg[button_idx].current_tick = 0;
			btn_cfg[button_idx].debounce_state = STATE_IDLE;
			k_timer_stop(buttons_timer[button_idx]);
			struct event_t event;
			LOG_INF("Button %d LONGGGGG pressed\r\n", button_idx);
			event.button_activity.button_pin = btn_cfg[button_idx].btn_pin;
			event.button_activity.button_action = BUTTON_LONG_PRESS;
			event.event_source = EVT_SRC_BUTTON;
			if (ctrl_events_queue_empty()) 
			{
				int ret = ctrl_events_put(&event);
				ERR_CHK(ret);
			} 
			else 
			{
				LOG_WRN("Event queue is not empty, try again later");
			}
		}
	}
	else
	{
		if(btn_cfg[button_idx].current_tick > BUTTON_DEBOUNCE && btn_cfg[button_idx].current_tick < BUTTON_LONGPRESS)
		{
			btn_cfg[button_idx].current_tick = 0;
			button_press_send_event[0](button_idx); 
		}
		else
		{
			//LOG_DBG("Button:%d - current tick:%d\r\n", button_idx, btn_cfg[button_idx].current_tick);
			if(btn_cfg[button_idx].button_multiple_press_count)
			{
				//btn_cfg[button_idx].time_between_press;
				if(1)
				{
					btn_cfg[button_idx].time_between_press = 0;
					btn_cfg[button_idx].button_multiple_press_count++;
					if(btn_cfg[button_idx].button_multiple_press_count >= 3)
					{
						btn_cfg[button_idx].button_multiple_press_count = 3;
					}
					button_press_send_event[btn_cfg[button_idx].button_multiple_press_count - 1](button_idx);
					btn_cfg[button_idx].button_multiple_press_count = 0;
					btn_cfg[button_idx].debounce_state = STATE_IDLE;
					k_timer_stop(buttons_timer[button_idx]);
				}
			}
			else
			{
			//memset(&btn_cfg[button_idx], 0, sizeof(struct btn_config));
				btn_cfg[button_idx].current_tick = 0;
				btn_cfg[button_idx].debounce_state = STATE_IDLE;
				btn_cfg[button_idx].button_multiple_press_count = 0;
				btn_cfg[button_idx].current_status = true;
				btn_cfg[button_idx].time_between_press = 0;
				k_timer_stop(buttons_timer[button_idx]);
			}
		}
	}

}

/** @brief Find the index of a button from the pin number
 */
static int pin_to_btn_idx(uint8_t btn_pin, uint32_t *pin_idx)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(btn_cfg); i++) {
		if (btn_pin == btn_cfg[i].btn_pin) {
			*pin_idx = i;
			return 0;
		}
	}
	LOG_WRN("Button idx not found");
	return -ENODEV;
}

/** @brief Convert from mask to pin
 *
 * @note: Will check that a single bit and a single bit only is set in the mask.
 */
static int pin_msk_to_pin(uint32_t pin_msk, uint32_t *pin_out)
{
	if (!pin_msk) {
		LOG_ERR("Mask is empty");
		return -EACCES;
	}

	if (pin_msk & (pin_msk - 1)) {
		LOG_ERR("Two or more buttons set in mask");
		return -EACCES;
	}

	*pin_out = 0;

	while (pin_msk) {
		pin_msk = pin_msk >> 1;
		(*pin_out)++;
	}

	/* Deduct 1 for zero indexing */
	(*pin_out)--;

	return 0;
}

/*  ISR triggered by GPIO when assigned button(s) are pushed */
static void button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pin_msk)
{
	int ret;
	struct event_t event;
	if (debounce_is_ongoing) {
		LOG_WRN("Btn debounce in action");
		return;
	}

	uint32_t btn_pin = 0;
	uint32_t btn_idx = 0;

	ret = pin_msk_to_pin(pin_msk, &btn_pin);
	ERR_CHK(ret);

	ret = pin_to_btn_idx(btn_pin, &btn_idx);
	ERR_CHK(ret);

	//LOG_INF("Pushed button idx: %d pin: %d name: %s", btn_idx, btn_pin,
	//	btn_cfg[btn_idx].btn_name);
	if(btn_cfg[btn_idx].debounce_state == STATE_IDLE)
	{
		btn_cfg[btn_idx].debounce_state = STATE_DEBOUCING;
		k_timer_start(buttons_timer[btn_idx], K_MSEC(1), K_MSEC(1));
	}
	// event.button_activity.button_pin = btn_pin;
	// event.button_activity.button_action = BUTTON_PRESS;
	// event.event_source = EVT_SRC_BUTTON;

	// /* To avoid filling up the event queue with button presses,
	//  * we only allow button events if all other events have been processed
	//  */
	// if (ctrl_events_queue_empty()) {
	// 	ret = ctrl_events_put(&event);
	// 	ERR_CHK(ret);
	// 	debounce_is_ongoing = true;
	// 	k_timer_start(&button_debounce_timer, K_MSEC(CONFIG_BUTTON_DEBOUNCE_MS), K_NO_WAIT);
	// } else {
	// 	LOG_WRN("Event queue is not empty, try again later");
	// }
}

int button_pressed(gpio_pin_t button_pin, bool *button_pressed)
{
	int ret;

	if (!device_is_ready(gpio_53_dev)) {
		return -ENODEV;
	}
	if (button_pressed == NULL) {
		return -EINVAL;
	}
	if(button_pin == BUTTON_VOLUME_UP)
	{
		ret = gpio_pin_get(gpio_54_dev, button_pin);
	}
	else
	{
		ret = gpio_pin_get(gpio_53_dev, button_pin);
	}
	switch (ret) {
	case 0:
		*button_pressed = false;
		break;
	case 1:
		*button_pressed = true;
		break;
	default:
		return ret;
	}

	return 0;
}

int button_handler_init(void)
{
	int ret;
	if (ARRAY_SIZE(btn_cfg) == 0) {
		LOG_WRN("No buttons assigned");
		return -EINVAL;
	}
	gpio_53_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	gpio_54_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	if(!device_is_ready(gpio_53_dev)) {
		LOG_ERR("Device driver not ready.");
		return -ENODEV;
	}
	if(!device_is_ready(gpio_54_dev))
	{
		LOG_ERR("Device driver not ready.");
		return -ENODEV;
	}
	for (uint8_t i = 0; i < ARRAY_SIZE(btn_cfg); i++) {
		if(i != 1)
		{
			ret = gpio_pin_configure(gpio_53_dev, btn_cfg[i].btn_pin,
					 GPIO_INPUT | btn_cfg[i].btn_cfg_mask);
		}
		else
		{
			ret = gpio_pin_configure(gpio_54_dev, btn_cfg[i].btn_pin,
			  		 GPIO_INPUT | btn_cfg[i].btn_cfg_mask);
		}
		if (ret) {
			return ret;
		}
		gpio_init_callback(&btn_callback[i], button_isr, BIT(btn_cfg[i].btn_pin));
		if(i != 1)
		{
			ret = gpio_add_callback(gpio_53_dev, &btn_callback[i]);
			if (ret) {
				return ret;
			}
			if(i != 3)
			{
				ret = gpio_pin_interrupt_configure(gpio_53_dev, btn_cfg[i].btn_pin, GPIO_INT_EDGE_FALLING);
			}
			else if (/* condition */i == 3)
			{
				/* code */
				ret = gpio_pin_interrupt_configure(gpio_53_dev, btn_cfg[i].btn_pin, GPIO_INT_EDGE_RISING);
			}
		}
		else
		{
			ret = gpio_add_callback(gpio_54_dev, &btn_callback[i]);
			if (ret) {
				return ret;
			}
		   	ret = gpio_pin_interrupt_configure(gpio_54_dev, btn_cfg[i].btn_pin, GPIO_INT_EDGE_FALLING);
		}
		if (ret) {
			return ret;
		}
	}
	return 0;
}

/* Shell functions */
static int cmd_print_all_btns(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (uint8_t i = 0; i < ARRAY_SIZE(btn_cfg); i++) {
		shell_print(shell, "Id %d: pin: %d %s", i, btn_cfg[i].btn_pin, btn_cfg[i].btn_name);
	}

	return 0;
}

static int cmd_push_btn(const struct shell *shell, size_t argc, char **argv)
{
	int ret;
	uint8_t btn_idx;
	struct event_t event;

	/* First argument is function, second is button idx */
	if (argc != 2) {
		shell_error(shell, "Wrong number of arguments provided");
		return -EINVAL;
	}

	if (!isdigit((int)argv[1][0])) {
		shell_error(shell, "Supplied argument is not numeric");
		return -EINVAL;
	}

	btn_idx = strtoul(argv[1], NULL, BASE_10);

	if (btn_idx >= ARRAY_SIZE(btn_cfg)) {
		shell_error(shell, "Selected button ID out of range");
		return -EINVAL;
	}

	event.button_activity.button_pin = btn_cfg[btn_idx].btn_pin;
	event.button_activity.button_action = BUTTON_PRESS;
	event.event_source = EVT_SRC_BUTTON;

	ret = ctrl_events_put(&event);

	if (ret == -ENOMSG) {
		LOG_WRN("Event queue is full, ignoring button press");
		ret = 0;
	}

	ERR_CHK(ret);

	shell_print(shell, "Pushed button idx: %d pin: %d : %s", btn_idx, btn_cfg[btn_idx].btn_pin,
		    btn_cfg[btn_idx].btn_name);

	return 0;
}

/* Creating subcommands (level 1 command) array for command "demo". */
SHELL_STATIC_SUBCMD_SET_CREATE(buttons_cmd,
			       SHELL_COND_CMD(CONFIG_SHELL, print, NULL, "Print all buttons.",
					      cmd_print_all_btns),
			       SHELL_COND_CMD(CONFIG_SHELL, push, NULL, "Push button.",
					      cmd_push_btn),
			       SHELL_SUBCMD_SET_END);
/* Creating root (level 0) command "demo" without a handler */
SHELL_CMD_REGISTER(buttons, &buttons_cmd, "List and push buttons", NULL);
