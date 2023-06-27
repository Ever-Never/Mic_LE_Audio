/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
// #include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>

#include <stdio.h>

#include "autoconf.h"
#include "ble_custom_nus.h"
#include <zephyr/logging/log.h>
#include "pair_ultilities.h"
#include "le_audio.h"
#include <zephyr/zbus/zbus.h>

//#include "ctrl_events.h"
#include "led.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE 1024
#define PRIORITY 7

#define DEVICE_PAIR_NAME "Pair LavalierMicrophone"
#define DEVICE_NAME DEVICE_PAIR_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE 64
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define CONFIG_PAIR_TIMEOUT_PUBLISH_STACK_SIZE	300
#define CONFIG_PAIR_TIMEOUT_PUBLISH_THREAD_PRIO 5


static struct bt_conn *current_conn;

static ble_custom_nus_fair_done_callback m_pair_done_fn = NULL;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static bool m_device_is_in_pair_mode = false;
static bool m_device_is_initialzed = false;


static void pair_timeout_publish(void);
static void pair_timer_handle(struct k_timer *timer);

/*"to" means timeout :))*/
ZBUS_CHAN_DEFINE(pair_peripheral_to_channel, struct nus_peripheral_event, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));
K_THREAD_DEFINE(pair_timeout_publish_thread_id, CONFIG_PAIR_TIMEOUT_PUBLISH_STACK_SIZE, pair_timeout_publish,
		NULL, NULL, NULL, K_PRIO_PREEMPT(CONFIG_PAIR_TIMEOUT_PUBLISH_THREAD_PRIO), 0, 0);

K_MSGQ_DEFINE(pair_to_queue, sizeof(struct nus_peripheral_event), 1, 4);

static void pair_timeout_publish(void)
{
	int ret;
	struct nus_peripheral_event msg;

	while (1) {
		k_msgq_get(&pair_to_queue, &msg, K_FOREVER);

		ret = zbus_chan_pub(&pair_peripheral_to_channel, &msg, K_NO_WAIT);
		if (ret) {
			LOG_ERR("Failed to publish button msg, ret: %d", ret);
		}
	}
}

K_TIMER_DEFINE(central_pair_timer, pair_timer_handle, NULL);
static void pair_timer_handle(struct k_timer *timer)
{
	/*if device hold connection*/
	int ret = 0;
	struct nus_peripheral_event event;
	event.event = NUS_PERIPHERAL_EVENT_PAIR_TIMEOUT;
	ret = k_msgq_put(&pair_to_queue, (void *)&event, K_NO_WAIT);
	if(ret == -EAGAIN)
	{
		LOG_WRN("Btn msg queue full");
	}
}
K_TIMER_DEFINE(pair_timer, pair_timer_handle, NULL);

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
	/*Uart Service Connect*/
	//dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	if(m_pair_done_fn)
	{
		/*
			Neu o bis Mode thi thiet bi khong can thiet phai nhan tin hieu tra lai :D
		*/
		/*Stop advertising pair msg*/
		ble_custom_nus_stop_pair();
		m_pair_done_fn(NULL);
		m_pair_done_fn = NULL;
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
//static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif
static void bt_rx_handle_wrong_msg_parser()
{
	//ble_custom_nus_stop_pair();
		/*if device hold connection*/
	if(current_conn)
	{
			/*Disconnect the device*/
		bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}

}
static void pair_info_message_create(uint8_t *p_out, uint8_t message_id)
{
	if(p_out == NULL)
	{
		LOG_ERR("Invalid output buffer");
	}
	pair_response_message_t rsp_msg;
	rsp_msg.reponse_msg_token = MIXER_DEFAULT_RESPONSE_TOKEN;
	memcpy(rsp_msg.gateway_mac, ultilities_get_mac(), 6);
	rsp_msg.device_type = DEVICE_TYPE_GATEWAY_MIXER;
	rsp_msg.message_id = message_id;
	
	memcpy(p_out, &rsp_msg, sizeof(pair_response_message_t));

}
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	uint8_t send_buffer[sizeof(pair_response_message_t) + 1];
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);
	LOG_HEXDUMP_INF(data, len, "Receive data \r\n");
	if(len != sizeof(pair_info_request_key_t))
	{
		bt_rx_handle_wrong_msg_parser();
		LOG_ERR("Wrong Pair message length\r\n");
		return;
	}
	pair_info_request_key_t request_key;
	memcpy(&request_key, data, sizeof(pair_info_request_key_t));
	pair_info_request_key_t *p_request_key = &request_key;
	if(p_request_key->token != MIXER_DEFAULT_TOKEN)
	{	
		bt_rx_handle_wrong_msg_parser();
		LOG_ERR("WRONG Default Token\r\n");
		return;
	}
	if(p_request_key->msg_id != MIXER_MESSAGE_ID_PAIR)
	{
		bt_rx_handle_wrong_msg_parser();
		LOG_ERR("Wrong message ID\r\n");
		return;
	}
	if(p_request_key->device_type != DEVICE_TYPE_SPEAKER)
	{
		bt_rx_handle_wrong_msg_parser();
		LOG_ERR("Invalid Device\r\n");
		return;
	}
	pair_info_message_create(send_buffer, MIXER_MESSAGE_ID_RESPONSE_PAIR);
	LOG_HEXDUMP_INF(send_buffer, sizeof(send_buffer), "Send to \r\n");
	int ret = bt_nus_send(conn, send_buffer, sizeof(pair_info_request_key_t));
	/**/
	app_led_indicator_pair_success();
	if(ret)
	{
		LOG_ERR("Send pair reponse message fail\r\n");
		bt_rx_handle_wrong_msg_parser();
		return;
	}
	else
	{
		/*if device hold connection*/
		if(current_conn)
		{
			/*Disconnect the device*/
			k_timer_stop(&pair_timer);
			bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

void ble_custom_nus_init(void)
{
	if(m_device_is_initialzed)
	{
		LOG_WRN("NUS server pair peripheral already init\r\n");
		return;
	}
	int err = 0;
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) 
	{ 
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}
	m_device_is_initialzed = true;
}
int ble_custom_nus_start_pair(ble_custom_nus_fair_done_callback p_pair_done_callback)
{
	if(m_device_is_in_pair_mode != true)
	{
		m_pair_done_fn = p_pair_done_callback;
		int err = 0;
		/*start to monitor timeout*/
		k_timer_start(&pair_timer, K_SECONDS(20), K_NO_WAIT);
		err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		m_device_is_in_pair_mode = true;
		if (err) 
		{
			LOG_ERR("Advertising failed to start (err %d)", err);
			return err;
		}
	}
	else
	{
		LOG_ERR("Device is already in PAIR mode\r\n");
	}
	return 0;
}
int ble_custom_nus_stop_pair(void)
{
	if(m_device_is_in_pair_mode == true)
	{
		int err = 0;
		//k_timer_stop(&pair_timer);
		err = bt_le_adv_stop();

		m_device_is_in_pair_mode = false;
		if(current_conn)
		{	
		/*Disconnect the device*/
			bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		if(err)
		{
			LOG_ERR("Advertising nus service fail to stop (err %d)", err);
			return err;
		}
	}
	else
	{
		LOG_ERR("Device is not in pair mode\r\n");
	}
	return 0;
}
bool ble_custom_nus_is_in_pair_mode(void)
{
	return m_device_is_in_pair_mode;
}

