/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <zephyr/kernel.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
//#include <sys_clock.h>
#include <zephyr/debug/stack.h>

#include "ctrl_events.h"
#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
#include "button_handler.h"
#include "data_fifo.h"
#include "board.h"
#include "le_audio.h"
#include "audio_datapath.h"
#include "audio_sync_timer.h"
#include <zephyr/logging/log.h>
#include "ble_custom_nus.h"
#include "ble_custom_nus_central.h"
#include "pair_ultilities.h"
#include "hw_output.h"
#include "dfu_entry.h"
#include "battery_measurement.h"

LOG_MODULE_REGISTER(streamctrl, CONFIG_STREAMCTRL_LOG_LEVEL);

struct ble_iso_data {
	uint8_t data[CONFIG_BT_ISO_RX_MTU];
	size_t data_size;
	bool bad_frame;
	uint32_t sdu_ref;
	uint32_t recv_frame_ts;
} __packed;

DATA_FIFO_DEFINE(ble_fifo_rx, CONFIG_BUF_BLE_RX_PACKET_NUM, WB_UP(sizeof(struct ble_iso_data)));

static struct k_thread audio_datapath_thread_data;
static k_tid_t audio_datapath_thread_id;
K_THREAD_STACK_DEFINE(audio_datapath_thread_stack, CONFIG_AUDIO_DATAPATH_STACK_SIZE);

static enum stream_state strm_state = STATE_PAUSED;

typedef void (*button_pressed_cb_handler)(button_pin_t button_pin);

#if (CONFIG_BLE_ISO_TEST_PATTERN)

struct iso_recv_stats {
	uint32_t iso_rx_packets_received;
	uint32_t iso_rx_packets_lost;
};

static struct iso_recv_stats stats_overall;

/* Print statistics from test pattern run */
static void stats_print(char const *const name, struct iso_recv_stats const *const stats)
{
	uint32_t total_packets;

	total_packets = stats->iso_rx_packets_received + stats->iso_rx_packets_lost;

	LOG_INF("%s: Received %u/%u (%.2f%%) - Lost %u", name, stats->iso_rx_packets_received,
		total_packets, (float)stats->iso_rx_packets_received * 100 / total_packets,
		stats->iso_rx_packets_lost);
}

/* Separate function for assessing link quality using a test pattern sent from gateway */
static void ble_test_pattern_receive(uint8_t const *const p_data, size_t data_size, bool bad_frame)
{
	uint32_t total_packets;
	static uint8_t expected_packet_value;

	stats_overall.iso_rx_packets_received++;

	if (bad_frame) {
		LOG_WRN("Received bad frame");
	}

	total_packets = stats_overall.iso_rx_packets_received + stats_overall.iso_rx_packets_lost;

	/* Only check first value in packet */
	if (p_data[0] == expected_packet_value) {
		expected_packet_value++;
	} else {
		/* First packet will always be a mismatch
		 * if gateway hasn't been reset before connection
		 */
		if (stats_overall.iso_rx_packets_received != 1) {
			stats_overall.iso_rx_packets_lost++;
			LOG_WRN("Missing packet: value: %d, expected: %d", p_data[0],
				expected_packet_value);
		}

		expected_packet_value = p_data[0] + 1;
	}

	if ((total_packets % 100) == 0) {
		stats_print("Overall ", &stats_overall);
	}
}
#endif /* (CONFIG_BLE_ISO_TEST_PATTERN) */

/* Callback for handling BLE RX */
static void le_audio_rx_data_handler(uint8_t const *const p_data, size_t data_size, bool bad_frame,
				     uint32_t sdu_ref)
{
	/* Capture timestamp of when audio frame is received */
	uint32_t recv_frame_ts = audio_sync_timer_curr_time_get();

	/* Since the audio datapath thread is preemptive, no actions on the
	 * FIFO can happen whilst in this handler.
	 */

	if (strm_state != STATE_STREAMING) {
		/* Throw away data */
		LOG_DBG("Not in streaming state, throwing data: %d", strm_state);
		return;
	}

	int ret;
	struct ble_iso_data *iso_received = NULL;

#if (CONFIG_BLE_ISO_TEST_PATTERN)
	ble_test_pattern_receive(p_data, data_size, bad_frame);
	return;
#endif /* (CONFIG_BLE_ISO_TEST_PATTERN) */

	uint32_t blocks_alloced_num, blocks_locked_num;

	ret = data_fifo_num_used_get(&ble_fifo_rx, &blocks_alloced_num, &blocks_locked_num);
	ERR_CHK(ret);

	if (blocks_alloced_num >= CONFIG_BUF_BLE_RX_PACKET_NUM) {
		/* FIFO buffer is full, swap out oldest frame for a new one */

		void *stale_data;
		size_t stale_size;

		LOG_WRN("BLE ISO RX overrun");

		ret = data_fifo_pointer_last_filled_get(&ble_fifo_rx, &stale_data, &stale_size,
							K_NO_WAIT);
		ERR_CHK(ret);

		data_fifo_block_free(&ble_fifo_rx, &stale_data);
	}

	ret = data_fifo_pointer_first_vacant_get(&ble_fifo_rx, (void *)&iso_received, K_NO_WAIT);
	ERR_CHK_MSG(ret, "Unable to get FIFO pointer");

	if (data_size > ARRAY_SIZE(iso_received->data)) {
		ERR_CHK_MSG(-ENOMEM, "Data size too large for buffer");
		return;
	}

	memcpy(iso_received->data, p_data, data_size);

	iso_received->bad_frame = bad_frame;
	iso_received->data_size = data_size;
	iso_received->sdu_ref = sdu_ref;
	iso_received->recv_frame_ts = recv_frame_ts;

	ret = data_fifo_block_lock(&ble_fifo_rx, (void *)&iso_received,
				   sizeof(struct ble_iso_data));
	ERR_CHK_MSG(ret, "Failed to lock block");
}

/* Thread to receive data from BLE through a k_fifo and send to audio datapath */
static void audio_datapath_thread(void *dummy1, void *dummy2, void *dummy3)
{
	int ret;
	struct ble_iso_data *iso_received = NULL;
	size_t iso_received_size;

	while (1) {
		ret = data_fifo_pointer_last_filled_get(&ble_fifo_rx, (void *)&iso_received,
							&iso_received_size, K_FOREVER);
		ERR_CHK(ret);

#if ((CONFIG_AUDIO_DEV == GATEWAY) && (CONFIG_AUDIO_SOURCE_USB))
		ret = audio_decode(iso_received->data, iso_received->data_size,
				   iso_received->bad_frame);
		ERR_CHK(ret);
#else
		audio_datapath_stream_out(iso_received->data, iso_received->data_size,
					  iso_received->sdu_ref, iso_received->bad_frame,
					  iso_received->recv_frame_ts);
#endif
		data_fifo_block_free(&ble_fifo_rx, (void *)&iso_received);

		STACK_USAGE_PRINT("audio_datapath_thread", &audio_datapath_thread_data);
	}
}

/* Function for handling all stream state changes */
static void stream_state_set(enum stream_state stream_state_new)
{
	strm_state = stream_state_new;
}

uint8_t stream_state_get(void)
{
	return strm_state;
}

void streamctrl_encoded_data_send(void const *const data, size_t len)
{
	int ret;
	static int prev_ret;

	if (strm_state == STATE_STREAMING) {
		ret = le_audio_send(data, len);

		if (ret != 0 && ret != prev_ret) {
			LOG_WRN("Problem with sending LE audio data, ret: %d", ret);
		}
		prev_ret = ret;
	}
}

/*
	@brief: Handling button pressed event
*/
static void button_pressed_event_handler(button_pin_t button_pin)
{
	uint8_t ret = 0;
	switch (button_pin) {
	case LINE_IN_DET:
	{
		bool line_in = app_button_read_detect_mic();
		LOG_INF("Detect line in stt: %d", line_in);
		hw_codec_set_input(line_in);
	}
	break;
	case BUTTON_PAIR:
	{
		LOG_INF("Button Pair");
#if(CONFIG_AUDIO_DEV == GATEWAY && IS_ENABLED(CONFIG_TRANSPORT_CIS))
		ret = le_audio_start_scan();
#elif(CONFIG_AUDIO_DEV == HEADSET && IS_ENABLED(CONFIG_TRANSPORT_CIS))
				//start adversting if slave :D

#endif
		
		//ERR_CHK(ret);
#if(CONFIG_AUDIO_DEV == GATEWAY && IS_ENABLED(CONFIG_TRANSPORT_BIS))
		ret = le_audio_disable();
		ret = ble_custom_nus_start_pair(le_audio_enable);
#elif(CONFIG_AUDIO_DEV == HEADSET && IS_ENABLED(CONFIG_TRANSPORT_BIS))
		ret = ble_custom_nus_central_start_find_pair_device(le_audio_enable);
#endif
		//ERR_CHK(ret);
		if(ret){
			LOG_WRN("Button pressed failed\r\n");		
		}
		app_led_indicator_in_pair();
		break;
	}

	case BUTTON_VOLUME_UP:
		hw_codec_volume_increase();
	break;
	
	case BUTTON_VOLUME_DOWN:
		hw_codec_volume_decrease();
	break;
	default:
		LOG_WRN("Unexpected/unhandled button id: %d", button_pin);
	}
}
static void button_long_pressed_event_handler(button_pin_t button_pin)
{
	switch (button_pin) {
		case BUTTON_PAIR:
		{
			/*TODO:Enter DFU mode*/
			//dfu_entry_check();
			break;
		}
		case BUTTON_ON_OFF:
		{
			//hw_output_set_level(GPIO_ON_OFF_CTRL, 1);
			while(app_button_read_onoff() == 0)
			{
				LOG_INF("Prepare to turn off device - turn off all leds and disable le audio");
				k_msleep(10);
				//le_audio_disable();
				led_off(LED_POWER);
				led_off(LED_BLE);
			}
			hw_output_button_handle();
		}
	}
}
static button_pressed_cb_handler m_button_handler[] = 
{
	button_pressed_event_handler,
	button_long_pressed_event_handler
};
/* Handle button activity events */
static void button_evt_handler(struct button_evt event)
{
	//int ret;

	LOG_DBG("Got btn evt from queue - id = %d, action = %d", event.button_pin,
		event.button_action);

	if (event.button_action > BUTTON_LONG_PRESS) {
		LOG_WRN("Unhandled button action");
		return;
	}
	if(m_button_handler[event.button_action])
	{
		m_button_handler[event.button_action](event.button_pin);
		return;
	}
}

/* Handle Bluetooth LE audio events */
static void le_audio_evt_handler(enum le_audio_evt_type event)
{
	int ret;
	LOG_DBG("Received event = %d, current state = %d", event, strm_state);
	switch (event) {
	case LE_AUDIO_EVT_STREAMING:
		LOG_INF("LE audio evt streaming");

		if (strm_state == STATE_STREAMING) {
			LOG_DBG("Got streaming event in streaming state");
			break;
		}
		audio_system_start();
		stream_state_set(STATE_STREAMING);
#if(1)
		//if(app_led_is_on(LED_POWER)||app_led_is_on(LED_BLE))
		//{
			//k_sleep(K_MSEC(100));
		//}
		//app_led_indicator_stop_both();
		app_led_indicator_streaming();
		//ERR_CHK(ret);
#endif
		break;

	case LE_AUDIO_EVT_NOT_STREAMING:
		LOG_DBG("LE audio evt not_streaming");

		if (strm_state == STATE_PAUSED) {
			LOG_DBG("Got not_streaming event in paused state");
			break;
		}

		stream_state_set(STATE_PAUSED);
		audio_system_stop();
		//ret = led_on(LED_APP_1_BLUE);
		//ERR_CHK(ret);
		app_led_indicator_stop_streaming();

		break;

	case LE_AUDIO_EVT_CONFIG_RECEIVED:
		LOG_DBG("Config received");

		uint32_t bitrate;
		uint32_t sampling_rate;

		ret = le_audio_config_get(&bitrate, &sampling_rate);
		if (ret) {
			LOG_WRN("Failed to get config err:%d", ret);
			break;
		}
		LOG_DBG("Sampling rate: %d Hz", sampling_rate);
		LOG_DBG("Bitrate: %d kbps", bitrate);
		break;

	default:
		LOG_WRN("Unexpected/unhandled event: %d", event);
		break;
	}
}
static void nus_custom_central_evt_handler(nus_central_event_t event)
{
	switch(event)
	{
		case NUS_CENTRAL_EVENT_PAIR_START:
		/*No implimentation needed */
			break;
		case NUS_CENTRAL_EVENT_PAIR_TIMEOUT:
			LOG_WRN("Pair timeout handle event\r\n");
			app_led_indicator_pair_timeout();
#if(CONFIG_AUDIO_DEV == HEADSET && IS_ENABLED(CONFIG_TRANSPORT_BIS))
			ble_custome_nus_central_stop_find_pair_device();
#elif(CONFIG_AUDIO_DEV == GATEWAY && IS_ENABLED(CONFIG_TRANSPORT_BIS))
			ble_custom_nus_stop_pair();
			le_audio_enable(NULL);
#endif
			break;
		default:
			LOG_WRN("Invalid custom nus event:%d\r\n", event);
			break;
	}
}
static void adc_eve_handler(battery_event_t event)
{
	switch(event)
	{
		case BATTERY_EVENT_START:
		/*No implimentation needed */
			break;
		case BATTERY_EVENT_MEASURE:
			int16_t battery_value = 0;
			battery_raw_data_get(&battery_value);
			break;
		default:
			LOG_WRN("Invalid custom nus event:%d\r\n", event);
			break;
	}
}
void streamctrl_event_handler(void)
{
	struct event_t my_event;

	/* As long as this timeout is K_FOREVER, ctrl_events_get should
	 * never return unless it has an event, that is why we can ignore the
	 * return value
	 */
	(void)ctrl_events_get(&my_event, K_FOREVER); /*Shitty this wait for event -> so every code no to be event loops :(*/

	switch (my_event.event_source) {
	case EVT_SRC_BUTTON:
		button_evt_handler(my_event.button_activity);
		break;
	case EVT_SRC_LE_AUDIO:
		le_audio_evt_handler(my_event.le_audio_activity.le_audio_evt_type);
		break;
	case EVT_SRC_TIMER:
		nus_custom_central_evt_handler(my_event.nus_central_activity.event);
		break;
	case EVT_SRC_ADC:
		adc_eve_handler(my_event.battery_activity.event);
		break;
	default:
		LOG_WRN("Unhandled event from queue - source = %d", my_event.event_source);
		break;
	}
	//uint16_t data;
	//battery_raw_data_get(&battery_raw_data_get);
}
/*Control pair state and stream state*/
int streamctrl_device_state()
{
	return 0;
}

int streamctrl_start(void)
{
	int ret;

	ret = data_fifo_init(&ble_fifo_rx);
	ERR_CHK_MSG(ret, "Failed to set up ble_rx FIFO");

	audio_datapath_thread_id =
		k_thread_create(&audio_datapath_thread_data, audio_datapath_thread_stack,
				CONFIG_AUDIO_DATAPATH_STACK_SIZE,
				(k_thread_entry_t)audio_datapath_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(CONFIG_AUDIO_DATAPATH_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(audio_datapath_thread_id, "AUDIO DATAPATH");
	ERR_CHK(ret);

	ret = le_audio_enable(le_audio_rx_data_handler);
	ERR_CHK_MSG(ret, "Failed to enable LE Audio");

	return 0;
}
