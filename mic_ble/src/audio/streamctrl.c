/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/debug/stack.h>
#include <zephyr/zbus/zbus.h>

#include "nrf5340_audio_common.h"
#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
#include "button_handler.h"
#include "data_fifo.h"
#include "board.h"
#include "le_audio.h"
#include "audio_datapath.h"

#include "battery_measurement.h"
#include <zephyr/logging/log.h>


#include "hw_output.h"
#include "ble_custom_nus.h"
#include "ble_custom_nus_central.h"
#include "pair_ultilities.h" 
#include "button_handler.h"
#include "hw_codec.h"




#define CONFIG_BATTERY_MEASURE_MENT_SUB_STACK_SIZE	512
#define CONFIG_PAIR_TIMEOUT_SUB_STACK_SIZE	512
#define CONFIG_AUDIO_STATE_MACHINE_SUB_STACK_SIZE 2048

LOG_MODULE_REGISTER(streamctrl, CONFIG_STREAMCTRL_LOG_LEVEL);

struct ble_iso_data {
	uint8_t data[CONFIG_BT_ISO_RX_MTU];
	size_t data_size;
	bool bad_frame;
	uint32_t sdu_ref;
	uint32_t recv_frame_ts;
} __packed;
typedef void (*button_pressed_cb_handler)(button_pin_t button_pin);


DATA_FIFO_DEFINE(ble_fifo_rx, CONFIG_BUF_BLE_RX_PACKET_NUM, WB_UP(sizeof(struct ble_iso_data)));

ZBUS_SUBSCRIBER_DEFINE(button_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
ZBUS_SUBSCRIBER_DEFINE(le_audio_evt_sub, CONFIG_LE_AUDIO_MSG_SUB_QUEUE_SIZE);
ZBUS_SUBSCRIBER_DEFINE(battery_measure_sub, 1);
ZBUS_SUBSCRIBER_DEFINE(audio_state_sub, 10);


static struct k_thread audio_datapath_thread_data;
static struct k_thread button_msg_sub_thread_data;
static struct k_thread le_audio_msg_sub_thread_data;
static struct k_thread battery_measurement_thread_data;
#if(CONFIG_AUDIO_DEV == HEADSET)
static struct k_thread audio_state_machine_thread_data;
#endif

static k_tid_t audio_datapath_thread_id;
static k_tid_t button_msg_sub_thread_id;
static k_tid_t le_audio_msg_sub_thread_id;
static k_tid_t battery_measurement_thread_id;
#if(CONFIG_AUDIO_DEV == HEADSET)
static k_tid_t audio_state_machine_thread_id;
#endif


K_THREAD_STACK_DEFINE(audio_datapath_thread_stack, CONFIG_AUDIO_DATAPATH_STACK_SIZE);
K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(le_audio_msg_sub_thread_stack, CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(battery_measurement_thread_stack, CONFIG_BATTERY_MEASURE_MENT_SUB_STACK_SIZE);
#if(CONFIG_AUDIO_DEV == HEADSET)
K_THREAD_STACK_DEFINE(audio_state_machine_thread_stack, CONFIG_AUDIO_STATE_MACHINE_SUB_STACK_SIZE);
#endif
/*Define paring thread timeout monitor for device*/
static struct k_thread pair_timeout_thread_data;
static k_tid_t pair_timeout_thread_id;
K_THREAD_STACK_DEFINE(pair_timeout_thread_stack, CONFIG_PAIR_TIMEOUT_SUB_STACK_SIZE);

#if(CONFIG_AUDIO_DEV == 2)




ZBUS_SUBSCRIBER_DEFINE(pair_peripheral_sub, 4);
#else


#define CONFIG_AUDIO_STATE_PUBLISH_STACK_SIZE	1024
#define CONFIG_AUDIO_STATE_PUBLISH_THREAD_PRIO 	5


static void audio_state_publish_thread(void);

ZBUS_CHAN_DEFINE(audio_state_channel, audio_state_data_t, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));
K_THREAD_DEFINE(audio_state_publish_id, CONFIG_AUDIO_STATE_PUBLISH_STACK_SIZE, audio_state_publish_thread,
		NULL, NULL, NULL, K_PRIO_PREEMPT(CONFIG_AUDIO_STATE_PUBLISH_THREAD_PRIO), 0, 0);
/* Only allow one button msg at a time, as a mean of debounce */
K_MSGQ_DEFINE(audio_state_queue, sizeof(audio_state_data_t), 1, 16);


ZBUS_SUBSCRIBER_DEFINE(pair_central_sub, 4);


#endif
static enum stream_state strm_state = STATE_PAUSED;



struct rx_stats {
	uint32_t recv_cnt;
	uint32_t bad_frame_cnt;
	uint32_t data_size_mismatch_cnt;
};


/*Function Forward declarations*/
static void le_audio_evt_handler(enum le_audio_evt_type event);
static void button_pressed_event_handler(button_pin_t button_pin);
static void button_long_pressed_event_handler(button_pin_t button_pin);
static void button_evt_handler(struct button_evt event);
static void adc_eve_handler(battery_event_t event);





static button_pressed_cb_handler m_button_handler[] = 
{
	button_pressed_event_handler,
	button_long_pressed_event_handler
};
#if(CONFIG_AUDIO_DEV == HEADSET)
int audio_state_publish_event(audio_state_data_t audio_state)
{
	int ret = 0;
	ret = k_msgq_put(&audio_state_queue, (void *)&audio_state, K_NO_WAIT);
	if(ret == -EAGAIN)
	{
		LOG_WRN("Audio state queue full");
	}
}
static void audio_state_publish_thread(void)
{
	int ret;
	audio_state_data_t msg;

	while (1) {
		k_msgq_get(&audio_state_queue, &msg, K_FOREVER);

		ret = zbus_chan_pub(&audio_state_channel, &msg, K_NO_WAIT);
		if (ret) {
			LOG_ERR("Failed to publish audio event, ret: %d", ret);
		}
	}
}
#endif
/* Callback for handling BLE RX */
static void le_audio_rx_data_handler(uint8_t const *const p_data, size_t data_size, bool bad_frame,
				     uint32_t sdu_ref, enum audio_channel channel_index,
				     size_t desired_data_size)
{
	/* Capture timestamp of when audio frame is received */
	uint32_t recv_frame_ts = nrfx_timer_capture(&audio_sync_timer_instance,
						    AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);

	/* Since the audio datapath thread is preemptive, no actions on the
	 * FIFO can happen whilst in this handler.
	 */

	bool data_size_mismatch = false;
	static struct rx_stats rx_stats[AUDIO_CH_NUM];

	rx_stats[channel_index].recv_cnt++;
	if (data_size != desired_data_size && !bad_frame) {
		data_size_mismatch = true;
		rx_stats[channel_index].data_size_mismatch_cnt++;
	}

	if (bad_frame) {
		rx_stats[channel_index].bad_frame_cnt++;
	}

	if ((rx_stats[channel_index].recv_cnt % 100) == 0 && rx_stats[channel_index].recv_cnt) {
		LOG_DBG("ISO RX SDUs: Ch: %d Total: %d Bad: %d Size mismatch %d", channel_index,
			rx_stats[channel_index].recv_cnt, rx_stats[channel_index].bad_frame_cnt,
			rx_stats[channel_index].data_size_mismatch_cnt);
	}

	if (data_size_mismatch) {
		/* Return if sizes do not match */
		return;
	}

	if (strm_state != STATE_STREAMING) {
		/* Throw away data */
		LOG_DBG("Not in streaming state, throwing data: %d", strm_state);
		return;
	}

	int ret;
	struct ble_iso_data *iso_received = NULL;

#if (CONFIG_AUDIO_DEV == GATEWAY)
	if (channel_index != AUDIO_CH_L) {
		/* Only left channel RX data in use on gateway */
		return;
	}
#endif /* (CONFIG_AUDIO_DEV == GATEWAY) */

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

void streamctrl_encoded_data_send(void const *const data, size_t size, uint8_t num_ch)
{
	int ret;
	static int prev_ret;

	struct encoded_audio enc_audio = { .data = data, .size = size, .num_ch = num_ch };

	if (strm_state == STATE_STREAMING) {
		ret = le_audio_send(enc_audio);

		if (ret != 0 && ret != prev_ret) {
			LOG_WRN("Problem with sending LE audio data, ret: %d", ret);
		}
		prev_ret = ret;
	}
}

/* Handle button activity */
static void button_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&button_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct button_evt msg;

		ret = zbus_chan_read(chan, &msg, K_MSEC(100));
		ERR_CHK(ret);

		LOG_DBG("Got btn evt from queue - id = %d, action = %d", msg.button_pin,
			msg.button_action);
		button_evt_handler(msg);
		STACK_USAGE_PRINT("button_msg_thread", &button_msg_sub_thread_data);
	}
}

/* Handle Bluetooth LE audio events */

static void le_audio_msg_sub_thread(void)
{
	int ret;
	uint32_t bitrate_bps;
	uint32_t sampling_rate_hz;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&le_audio_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct le_audio_msg msg;

		ret = zbus_chan_read(chan, &msg, K_MSEC(100));
		ERR_CHK(ret);
		le_audio_evt_handler(msg.event);
		STACK_USAGE_PRINT("le_audio_msg_thread", &le_audio_msg_sub_thread_data);
	}
}

static void battery_measurement_sub_thread(void)
{
	int ret = 0;
	const struct zbus_channel *chan;
	while(1)
	{
		ret = zbus_sub_wait(&battery_measure_sub, &chan, K_FOREVER);
		ERR_CHK(ret);
		struct battery_event msg;
		ret = zbus_chan_read(chan, &msg, K_MSEC(100));
		adc_eve_handler(msg.event);
		STACK_USAGE_PRINT("battery measurement thread", &battery_measurement_thread_data);
	}
}
static void pair_timeout_sub_thread(void)
{
	int ret = 0;
	const struct zbus_channel *chan;
	while(1)
	{
#if(CONFIG_AUDIO_DEV == GATEWAY)
		ret = zbus_sub_wait(&pair_peripheral_sub, &chan, K_FOREVER);
		ERR_CHK(ret);
		struct nus_peripheral_event msg;
#elif(CONFIG_AUDIO_DEV == HEADSET)
		ret = zbus_sub_wait(&pair_central_sub, &chan, K_FOREVER);
		ERR_CHK(ret);
#if(CONFIG_AUDIO_DEV == HEADSET)
		struct nus_central_event msg;

		ret = zbus_chan_read(chan, &msg, K_MSEC(100));
		switch(msg.event)
		{
			case NUS_CENTRAL_EVENT_PAIR_START:
			{
				//ret = ble_stop_scan_broadcast_source();
				//ret = le_audio_disable();
				//ret = ble_custom_nus_central_start_find_pair_device(le_audio_enable);

				audio_state_data_t audio_evt;
				audio_evt.evt = AUDIO_STATE_EVT_ENTER_PAIR;
				audio_evt.data_len = 0;
				audio_state_publish_event(audio_evt);
				app_led_indicator_in_pair();
			}
			break;
			case NUS_CENTRAL_EVENT_PAIR_TIMEOUT:
			{
				app_led_indicator_pair_timeout();
				ble_custome_nus_central_stop_find_pair_device();
				audio_state_data_t audio_evt;
				audio_evt.evt = AUDIO_STATE_EVT_EXIT_PAIR;
				audio_evt.data_len = 0;
				audio_state_publish_event(audio_evt);
			}
				break;
			case NUS_CENTRAL_EVENT_PAIR_SUCCESS:
			{
				app_led_indicator_pair_timeout();
				audio_state_data_t audio_evt;
				audio_evt.evt = AUDIO_STATE_EVT_EXIT_PAIR;
				audio_evt.data_len = 0;
				audio_state_publish_event(audio_evt);
			}
				break;
			default:
			break;
		}
#endif
#endif
		//pair_timeout(msg.event);
		STACK_USAGE_PRINT("battery measurement thread", &battery_measurement_thread_data);
	}
}

/*
* @brief: Valid the correct paired data
*/
static bool manufacture_data_validate(manu_ext_adv_data_t data)
{
	/*This had better priority* -> Gateway auto speak if the is other devicce is speaking*/
	if(data.refined.device_type == DEVICE_TYPE_GATEWAY_SUB)
	{
		for(uint8_t i = 0; i < pair_ultilities_gateway_pair_load()->refined.total_device; i++)
		{
			if(memcmp(pair_ultilities_gateway_pair_load()->refined.gateway_data[i].gw_refined.device_mac, data.refined.mac, 6) == 0)
			{
				pair_ultilities_gateway_pair_load()->refined.gateway_data[i].gw_refined.request_to_speak = data.refined.request_to_speak;
				if(data.refined.request_to_speak)
				{
					pair_ultilities_change_pair_gateway(data.refined.mac);
					return true;
				}
				else
				{
					return false;
				}
			}
		}
	}
	if(data.refined.device_type == DEVICE_TYPE_GATEWAY_MIXER)
	{
		for(uint8_t i = 0; i < pair_ultilities_gateway_pair_load()->refined.total_device; i++)
		{
			if(memcmp(pair_ultilities_gateway_pair_load()->refined.gateway_data[i].gw_refined.device_mac, data.refined.mac, 6) == 0)
			{
				pair_ultilities_change_pair_gateway(data.refined.mac);
				return true;
			}
		}
	}
	return false;

}
#if(CONFIG_AUDIO_DEV == HEADSET)
static void audio_state_machine_thread(void)
{
	static audio_state_t audio_state = AUDIO_STATE_INIT;
	int ret = 0;
	const struct zbus_channel *chan;
	while(1)
	{
		ret = zbus_sub_wait(&audio_state_sub, &chan, K_FOREVER);
		audio_state_data_t msg;
		ret = zbus_chan_read(chan, &msg, K_MSEC(100));
		if(msg.evt == AUDIO_STATE_EVT_ENTER_PAIR)
		{
			audio_state = AUDIO_STATE_PAIR_STATE;
			/**/
			ret = ble_stop_scan_broadcast_source();
			ret = le_audio_disable();
			ret = ble_custom_nus_central_start_find_pair_device(NULL);
		}
		switch(audio_state)
		{
			case AUDIO_STATE_INIT:
			{
				if(msg.evt == AUDIO_STATE_EVT_INIT)
				{	
					printk("Audio state INIT - evnt INIT\r\n");
					ret = le_audio_disable();
					audio_state = AUDIO_STATE_FIND_VALID_DEVICE;
					audio_state_data_t audio_evt;
					audio_evt.evt = AUDIO_STATE_EVT_START_ADV;
					audio_evt.data_len = 0;
					audio_state_publish_event(audio_evt);
				}
			}
			break;
			case AUDIO_STATE_FIND_VALID_DEVICE:
			{
				switch(msg.evt)
				{
					case AUDIO_STATE_EVT_START_ADV:
					{
						printk("Audio state FIND DEVICE - evnt START ADV\r\n");
						ret = ble_start_scan_vaid_broadcast_source();
					}
					break;
					case AUDIO_STATE_EVT_FOUND_VALID_SRC:
					{
						LOG_INF("Audio state FOUND VALID - evnt FOUND VALID SRC");

						/*Valid the valid source :D*/
						manu_ext_adv_data_t evt_data;
						memcpy(evt_data.raw, msg.data, msg.data_len);
						/*Check if device is pairded -> else don't care*/
						bool valid = manufacture_data_validate(evt_data);
						if(valid == 0)
						{
							printk("Not a paired gateway\r\n");
							break;;
						}			

						/**/
						ret = ble_stop_scan_broadcast_source();
						ret = le_audio_enable(NULL, NULL);
						if(ret)
						{
							printk("[%s]LE audio enable failed %d", ret, __FUNCTION__);
						}
						audio_state = AUDIO_STATE_SYNCING_SRC;
					}
					break;
				}
			}
			break;
			case AUDIO_STATE_SYNCING_SRC:
			{
				//ret = ble_stop_scan_broadcast_source();
				switch(msg.evt)
				{
					case AUDIO_STATE_EVT_RECEIVE_SRC_DATA:
					{
						audio_state = AUDIO_STATE_SYNC_AND_RECEIVE_SRC_DATA;
					}
					break;
					default:
					break;
				}
			}
			break;
			case AUDIO_STATE_SYNC_AND_RECEIVE_SRC_DATA:
			{
				switch(msg.evt)
				{
					case AUDIO_STATE_EVT_LOSS_SYNC:
						ret = le_audio_disable();
						if(ret)
						{
							printk("[%s]LE audio enable failed %d", ret, __FUNCTION__);
						}
						//audio_state = AUDIO_STATE_FIND_VALID_DEVICE;
						audio_state = AUDIO_STATE_FIND_VALID_DEVICE;
						audio_state_data_t audio_evt;
						audio_evt.evt = AUDIO_STATE_EVT_START_ADV;
						audio_evt.data_len = 0;
						audio_state_publish_event(audio_evt);
					break;
					case AUDIO_STATE_EVT_RECEIVE_SRC_DATA:
						break;
					default:
						break;
				}
			}
			break;
			case AUDIO_STATE_PAIR_STATE:
			{
				LOG_INF("AUdio PAIR STATE");
				if(msg.evt == AUDIO_STATE_EVT_EXIT_PAIR)
				{
					LOG_INF("Audio PAIR STATE - EXIT ENV");
					ble_custome_nus_central_stop_find_pair_device();
					audio_state = AUDIO_STATE_INIT;
					audio_state_data_t audio_evt;
					audio_evt.evt = AUDIO_STATE_EVT_INIT;
					audio_evt.data_len = 0;
					audio_state_publish_event(audio_evt);
				}
			}
			break;
			default:
				break;
		}
	}
}
#endif

int streamctrl_start(void)
{
	int ret;

	audio_system_init();

	ret = data_fifo_init(&ble_fifo_rx);
	ERR_CHK_MSG(ret, "Failed to set up ble_rx FIFO");

	button_msg_sub_thread_id =
		k_thread_create(&button_msg_sub_thread_data, button_msg_sub_thread_stack,
				CONFIG_BUTTON_MSG_SUB_STACK_SIZE,
				(k_thread_entry_t)button_msg_sub_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(button_msg_sub_thread_id, "BUTTON_MSG_SUB");
	ERR_CHK(ret);

	le_audio_msg_sub_thread_id =
		k_thread_create(&le_audio_msg_sub_thread_data, le_audio_msg_sub_thread_stack,
				CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE,
				(k_thread_entry_t)le_audio_msg_sub_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(CONFIG_LE_AUDIO_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(le_audio_msg_sub_thread_id, "LE_AUDIO_MSG_SUB");
	ERR_CHK(ret);

	audio_datapath_thread_id =
		k_thread_create(&audio_datapath_thread_data, audio_datapath_thread_stack,
				CONFIG_AUDIO_DATAPATH_STACK_SIZE,
				(k_thread_entry_t)audio_datapath_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(CONFIG_AUDIO_DATAPATH_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(audio_datapath_thread_id, "AUDIO DATAPATH");
	ERR_CHK(ret);

	battery_measurement_thread_id =
		k_thread_create(&battery_measurement_thread_data, battery_measurement_thread_stack,
				CONFIG_BATTERY_MEASURE_MENT_SUB_STACK_SIZE,
				(k_thread_entry_t)battery_measurement_sub_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	ret = k_thread_name_set(battery_measurement_thread_id, "BATTERY MEASUREMENT");
	ERR_CHK(ret);

	pair_timeout_thread_id =
		k_thread_create(&pair_timeout_thread_data, pair_timeout_thread_stack,
				CONFIG_PAIR_TIMEOUT_SUB_STACK_SIZE,
				(k_thread_entry_t)pair_timeout_sub_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	ret = k_thread_name_set(pair_timeout_thread_id, "PAIR TIMEOUT");
	ERR_CHK(ret);
#if(CONFIG_AUDIO_DEV == HEADSET)
	audio_state_machine_thread_id =
		k_thread_create(&audio_state_machine_thread_data, audio_state_machine_thread_stack,
				CONFIG_AUDIO_STATE_MACHINE_SUB_STACK_SIZE,
				(k_thread_entry_t)audio_state_machine_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	ret = k_thread_name_set(audio_state_machine_thread_id, "AUDIO STATE MACHINE");
	ERR_CHK(ret);
#endif
	ret = le_audio_enable(le_audio_rx_data_handler, audio_datapath_sdu_ref_update);
	ERR_CHK_MSG(ret, "Failed to enable LE Audio");
#if(CONFIG_AUDIO_DEV == HEADSET)
		// ret = le_audio_disable();
		// ret = ble_start_scan_vaid_broadcast_source();
	audio_state_data_t audio_evt;
	audio_evt.evt = AUDIO_STATE_EVT_INIT;
	audio_evt.data_len = 0;
	audio_state_publish_event(audio_evt);
#endif
	/*stop scanning for broadcast source*/
	return 0;
}



/* Handle Bluetooth LE audio events */
static void le_audio_evt_handler(enum le_audio_evt_type event)
{
	uint32_t pres_delay_us = 0;
	int ret;
	LOG_DBG("Received event = %d, current state = %d", event, strm_state);
	switch (event) {
	case LE_AUDIO_EVT_STREAMING:
		(void)pres_delay_us;
		LOG_INF("LE audio evt streaming");
		if (strm_state == STATE_STREAMING) {
			LOG_DBG("Got streaming event in streaming state");
			break;
		}
		audio_system_start();
		stream_state_set(STATE_STREAMING);
#if(1)
		app_led_indicator_streaming();
		
#endif
		break;
	case LE_AUDIO_EVT_NOT_STREAMING:
		LOG_DBG("LE audio evt not_streaming");
		(void)pres_delay_us;
		if (strm_state == STATE_PAUSED) {
			LOG_DBG("Got not_streaming event in paused state");
			break;
		}
		stream_state_set(STATE_PAUSED);
		audio_system_stop();
		app_led_indicator_stop_streaming();
		break;

	case LE_AUDIO_EVT_CONFIG_RECEIVED:
		LOG_DBG("Config received");
		uint32_t bitrate;
		uint32_t sampling_rate;
		//(void)pres_delay_us;
		ret = le_audio_config_get(&bitrate, &sampling_rate, &pres_delay_us);
		if (ret) {
			LOG_WRN("Failed to get config err:%d", ret);
			break;
		}
		LOG_DBG("Sampling rate: %d Hz", sampling_rate);
		LOG_DBG("Bitrate: %d kbps", bitrate);
			break;
	case LE_AUDIO_EVT_PRES_DELAY_SET:
		ret = le_audio_config_get(NULL, NULL, &pres_delay_us);
		if (ret) {
			LOG_ERR("Failed to get config");
			break;
		}
		ret = audio_datapath_pres_delay_us_set(pres_delay_us);
		if (ret) {
			LOG_ERR("Failed to set presentation delay to %d", pres_delay_us);
			break;
		}
		LOG_INF("Presentation delay %d us is set by initiator", pres_delay_us);
			break;
	default:
		LOG_WRN("Unexpected/unhandled event: %d", event);
		(void)pres_delay_us;
		break;
	}
}

static void button_long_pressed_event_handler(button_pin_t button_pin)
{
	switch (button_pin) {
		case BUTTON_PAIR:
		{
			/*TODO:Enter DFU mode*/
			break;
		}
		case BUTTON_ON_OFF:
		{
			while(app_button_read_onoff() == 0)
			{
				LOG_INF("Prepare to turn off device - turn off all leds and disable le audio");
				k_msleep(100);
				led_off(LED_POWER);
				led_off(LED_BLE);
			}
			ultilities_flash_set_reset_val(true);
			hw_output_button_handle();
		}
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
#if(CONFIG_AUDIO_DEV == GATEWAY && IS_ENABLED(CONFIG_TRANSPORT_BIS))
		ret = le_audio_disable();
		ret = ble_custom_nus_start_pair(le_audio_enable);
#elif(CONFIG_AUDIO_DEV == HEADSET && IS_ENABLED(CONFIG_TRANSPORT_BIS))
		struct nus_central_event evt = {0};
		evt.event = NUS_CENTRAL_EVENT_PAIR_START;
		ble_custom_nus_central_dispatch_evt(evt);
#endif
// 		ret = ble_custom_nus_central_start_find_pair_device(le_audio_enable);
// #endif
// 		if(ret){
// 			LOG_WRN("Button pressed failed\r\n");		
// 		}
		app_led_indicator_in_pair();
		break;
	}

	case BUTTON_VOLUME_UP:
		hw_codec_volume_increase();
		break;
	
	case BUTTON_VOLUME_DOWN:
		//ble_start_scan_vaid_broadcast_source();
		hw_codec_volume_decrease();
		break;

	default:
		LOG_WRN("Unexpected/unhandled button id: %d", button_pin);
	}
}

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