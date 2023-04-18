#ifndef __BLE_CUSTOM_NUS_H__
#define __BLE_CUSTOM_NUS_H__
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>
#include "le_audio.h"
#include "pair_ultilities.h"

void ble_custom_nus_init(void);

int ble_custom_nus_stop_pair(void);

int ble_custom_nus_start_pair(ble_custom_nus_fair_done_callback p_pair_done_callback);
#endif