/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/debug/stack.h>
#include <zephyr/device.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "fw_info_app.h"
#include "led.h"
#include "button_handler.h"
#include "button_assignments.h"
#include "nrfx_clock.h"
#include "ble_core.h"
#include "power_module.h"
#include "sd_card.h"
#include "audio_system.h"
#include "channel_assignment.h"
#include "streamctrl.h"
#include "hw_output.h"
#include "ble_custom_nus.h"
#include "battery_measurement.h"
#include "ble_custom_nus_central.h"
#include "debug/cpu_load.h"
#if defined(CONFIG_AUDIO_DFU_ENABLE)
#include "dfu_entry.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#if defined(CONFIG_INIT_STACKS)
/* Used for printing stack usage */
extern struct k_thread z_main_thread;
#endif /* defined(CONFIG_INIT_STACKS) */

static atomic_t ble_core_is_ready = (atomic_t) false;
static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

static int bonding_clear_check(void)
{
	// int ret;
	// bool pressed;

	// // ret = button_pressed(BUTTON_5, &pressed);
	// // if (ret) {
	// // 	return ret;
	// // }

	// if (pressed) {
	// 	if (IS_ENABLED(CONFIG_SETTINGS)) {
	// 		LOG_INF("Clearing all bonds");
	// 		bt_unpair(BT_ID_DEFAULT, NULL);
	// 	}
	// }
	return 0;
}

static int channel_assign_check(void)
{
#if (CONFIG_AUDIO_DEV == HEADSET) && CONFIG_AUDIO_HEADSET_CHANNEL_RUNTIME
	int ret;
	bool pressed;

	ret = button_pressed(BUTTON_VOLUME_DOWN, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		channel_assignment_set(AUDIO_CH_L);
		return 0;
	}

	ret = button_pressed(BUTTON_VOLUME_UP, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		channel_assignment_set(AUDIO_CH_R);
		return 0;
	}
#endif

	return 0;
}

/* Callback from ble_core when the ble subsystem is ready */
void on_ble_core_ready(void)
{
	int ret;

	(void)atomic_set(&ble_core_is_ready, (atomic_t) true);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();

		ret = bonding_clear_check();
		ERR_CHK(ret);
	}
}

void main(void)
{
	int ret;

	LOG_INF("nRF5340 APP core started");

	ret = hfclock_config_and_start();
	ERR_CHK(ret);

	ret = led_init();
	ERR_CHK(ret);
	app_led_turn_on_device();
	led_on(LED_POWER, 10);

	// ret = fw_info_app_print();
	// ERR_CHK(ret);
	ret = button_handler_init();
	ERR_CHK(ret);

	channel_assignment_init();

	ret = channel_assign_check();
	ERR_CHK(ret);

	ret = hw_output_init();
	ERR_CHK(ret);
	hw_output_set_level(GPIO_ON_OFF_CTRL, 1);
	
	ret = battery_measurement_init();
	ERR_CHK(ret);
#if defined(CONFIG_AUDIO_DFU_ENABLE)
	/* Check DFU BTN before Initialize BLE */
	dfu_entry_check();
#endif

	/* Initialize BLE, with callback for when BLE is ready */
	ret = ble_core_init(on_ble_core_ready);
	ERR_CHK(ret);

	/* Wait until ble_core/NET core is ready */
	while (!(bool)atomic_get(&ble_core_is_ready)) {
		(void)k_sleep(K_MSEC(100));
	}

	/*Init Nus custom for pairing feature*/
	ultilities_load_mac(); // load device mac address
	pair_ultilities_flash_init();
#if(CONFIG_AUDIO_DEV == 2 && IS_ENABLED(CONFIG_TRANSPORT_BIS))
	ble_custom_nus_init();
	pair_ultitlities_set_device_name(CONFIG_BT_DEVICE_NAME);
#elif(CONFIG_AUDIO_DEV == 1 && IS_ENABLED(CONFIG_TRANSPORT_BIS))
	ble_custom_nus_central_init();
	pair_ultilities_flash_init();
	uint8_t gw_if[6];
	if(pair_ultilities_flash_read_gateway_info(gw_if) == 0)
	{
		LOG_HEXDUMP_INF(gw_if, sizeof(gw_if), 6);
	}
	else 
	{
		LOG_WRN("NO gateway info");
		/* code */
	}
#endif
	audio_system_init();

	ret = streamctrl_start();
	cpu_load_init();
	while (1) {
		streamctrl_event_handler();
		STACK_USAGE_PRINT("main", &z_main_thread);
	}
}
