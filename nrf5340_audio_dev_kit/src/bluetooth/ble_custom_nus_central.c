/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include "ble_custom_nus_central.h"
#include "ctrl_events.h"
#include "macros_common.h"
#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* UART payload buffer element size. */
#define UART_BUF_SIZE 20
ble_custom_nus_fair_done_callback m_pair_done_callback = NULL;

#define NUS_WRITE_TIMEOUT K_MSEC(150)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50

struct uart_data_t {
	void *fifo_reserved;
	uint8_t  data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static struct bt_conn *default_conn;
static struct bt_nus_client nus_client;

static volatile bool m_device_is_in_pair_mode = false;

static void nus_client_create_pair_packet(uint8_t *p_data_out);
static void gatt_discover(struct bt_conn *conn);


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

	gatt_discover(conn);
}

static void pair_timer_handle(struct k_timer *timer)
{
	/*if device hold connection*/
	// if(default_conn)
	// {
	// 	/*Disconnect the device*/
	// 	bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	// }
	// else
	// {
		//ble_custome_nus_central_stop_find_pair_device();
		struct event_t nus;
		nus.nus_central_activity.event = NUS_CENTRAL_EVENT_PAIR_TIMEOUT;
		nus.event_source = EVT_SRC_TIMER;
		if(ctrl_events_queue_empty())
		{
			int ret = ctrl_events_put(&nus);
			ERR_CHK(ret);
		}
	//}
}
K_TIMER_DEFINE(central_pair_timer, pair_timer_handle, NULL);

static void ble_data_sent(struct bt_nus_client *nus, uint8_t err,
					const uint8_t *const data, uint16_t len)
{
	ARG_UNUSED(nus);
	if (err) {
		LOG_WRN("ATT error code: 0x%02X", err);
	}
}


static void discovery_complete(struct bt_gatt_dm *dm,
			       void *context)
{
	struct bt_nus_client *nus = context;
	LOG_INF("Service discovery completed");

	bt_gatt_dm_data_print(dm);

	bt_nus_handles_assign(dm, nus);
	bt_nus_subscribe_receive(nus);

	bt_gatt_dm_data_release(dm);

	uint8_t buffer_send[sizeof(pair_info_request_key_t)];
	nus_client_create_pair_packet(buffer_send);
	bt_nus_client_send(&nus_client, buffer_send, sizeof(buffer_send));
	LOG_HEXDUMP_WRN(buffer_send, sizeof(buffer_send), "Send pair message:");
}

static void discovery_service_not_found(struct bt_conn *conn,
					void *context)
{
	LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn,
			    int err,
			    void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err;

	if (conn != default_conn) {
		return;
	}
	err = bt_gatt_dm_start(conn,
			       BT_UUID_NUS_SERVICE,
			       &discovery_cb,
			       &nus_client);
	if (err) {
		LOG_ERR("could not start the discovery procedure, error "
			"code: %d", err);
	}
}

static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		LOG_INF("MTU exchange done");
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

/**/
static void nus_client_create_pair_packet(uint8_t *p_data_out)
{
	bt_addr_le_t addr;
	size_t size;
	pair_info_request_key_t msg_packet;
	bt_id_get(&addr, &size);
	msg_packet.token = MIXER_DEFAULT_TOKEN;
	msg_packet.msg_id = MIXER_MESSAGE_ID_PAIR;
	msg_packet.device_type = DEVICE_TYPE_SPEAKER;
	memcpy(msg_packet.mac, addr.a.val, 6);
	memcpy(p_data_out, &msg_packet, sizeof(msg_packet));
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		LOG_INF("Failed to connect to %s (%d)", addr, conn_err);

		if (default_conn == conn) {
			bt_conn_unref(default_conn);
			default_conn = NULL;

			// err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
			// if (err) {
			// 	LOG_ERR("Scanning failed to start (err %d)",
			// 		err);
			// }
		}

		return;
	}

	LOG_INF("Connected: %s", addr);

	static struct bt_gatt_exchange_params exchange_params;

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	}

	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		LOG_WRN("Failed to set security: %d", err);

		gatt_discover(conn);
	}
	else
	{
		LOG_INF("Set security success\r\n");
	}

	// err = bt_scan_stop();
	// if ((!err) && (err != -EALREADY)) {
	// 	LOG_ERR("Stop LE scan failed (err %d)", err);
	// }
	/*send pair message in hear*/

}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;
	ble_custome_nus_central_stop_find_pair_device();
}
static uint8_t ble_data_received(struct bt_nus_client *nus, const uint8_t *data, uint16_t len)
{
	ARG_UNUSED(nus);
	if(len != sizeof(pair_response_message_t))
	{
		LOG_ERR("Not fit pair response message length\r\n");
		return BT_GATT_ITER_CONTINUE;
	}
	pair_response_message_t pair_reponse;
	memcpy(&pair_reponse, data, sizeof(pair_info_request_key_t));
	if(pair_reponse.device_type != DEVICE_TYPE_GATEWAY_MIXER)
	{
		LOG_WRN("Invalid device type\r\n");
		return BT_GATT_ITER_CONTINUE;
	}
	if(pair_reponse.message_id != MIXER_MESSAGE_ID_RESPONSE_PAIR)
	{
		LOG_WRN("Invalid message ID\r\n");
		return BT_GATT_ITER_CONTINUE;
	}
	if(pair_reponse.reponse_msg_token != MIXER_DEFAULT_RESPONSE_TOKEN)
	{
		LOG_WRN("Invalid Token\r\n");
		return BT_GATT_ITER_CONTINUE;
	}
	pair_ultilities_flash_save_gateway_info(pair_reponse.gateway_mac);
	return BT_GATT_ITER_CONTINUE;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed
};
static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	LOG_INF("Filters matched. Address: %s connectable: %d",
		addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	LOG_WRN("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info,
			    struct bt_conn *conn)
{
	default_conn = bt_conn_ref(conn);
    /*TODO: Send pair message*/
}

static int nus_client_init(void)
{
	int err;
	struct bt_nus_client_init_param init = {
		.cb = {
			.received = ble_data_received,
			.sent = ble_data_sent,
		}
	};

	err = bt_nus_client_init(&nus_client, &init);
	if (err) {
		LOG_ERR("NUS Client initialization failed (err %d)", err);
		return err;
	}

	LOG_INF("NUS Client module initialized");
	return err;
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
		scan_connecting_error, scan_connecting);

static int scan_init(void)
{
	int err;
	struct bt_scan_init_param scan_init = {
		.connect_if_match = 1,
	};

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);
    /**/
	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, "Pair LavalierMicrophone");
	if (err) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		return err;
	}


	LOG_INF("Scan module initialized");
	return err;
}
static int scan_deinit()
{
	int err = 0;
	bt_scan_filter_disable();
	if (err) {
	LOG_ERR("Filters cannot be turned off (err %d)", err);
		return err;
	}
}


void ble_custom_nus_central_init(void)
{
	int err;

	LOG_INF("Bluetooth initialized central\r\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	int (*module_init[])(void) = {scan_init, nus_client_init};
	for (size_t i = 0; i < ARRAY_SIZE(module_init); i++) {
		err = (*module_init[i])();
		if (err) {
			return;
		}
	}
}

int ble_custom_nus_central_start_find_pair_device(ble_custom_nus_fair_done_callback p_pair_done_callback)
{
	int err = 0;
	if(m_device_is_in_pair_mode == false)
	{
		err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
		if (err) {
			LOG_ERR("Filters cannot be turned on (err %d)", err);
			return err;
		}
		else
		{
			LOG_INF("Turn on scan filter");
		}
		k_timer_start(&central_pair_timer, K_SECONDS(20), K_NO_WAIT);
		m_pair_done_callback = p_pair_done_callback;
		LOG_INF("Starting NUS pairing central\n");
		err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
		if (err) {
			LOG_ERR("Scanning failed to start (err %d)", err);
			return err;
		}
		LOG_INF("Scanning for pair device");
		m_device_is_in_pair_mode = true;
	}
	else
	{
		LOG_ERR("Device central is altreay in pair mode");
		return err;
	}
}
void ble_custome_nus_central_stop_find_pair_device()
{
	int err = 0;
	if(m_device_is_in_pair_mode == true)
	{
		
		if(m_pair_done_callback)
		{
			m_pair_done_callback(NULL);
			m_pair_done_callback = NULL;
		}
		//err = bt_scan_stop();
		scan_deinit();
		if(err)
		{
			LOG_ERR("Scanning error to stop (err :%d)\r\n", err);
			return;
		}
		//k_timer_stop(&central_pair_timer);
		if(default_conn)
		{
			bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
		m_device_is_in_pair_mode = false;
	}
	else 
	{
		LOG_ERR("Device central is free => Stop for wat ?");
	}	
}

bool ble_custom_nus_central_is_in_pair_mode()
{
	return m_device_is_in_pair_mode;
}