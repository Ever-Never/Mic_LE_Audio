#ifndef _BLE_CUSTOM_NUS_CENTRAL_
#define _BLE_CUSTOM_NUS_CENTRAL_
#include <stdio.h>
#include <stdint.h>
#include "pair_ultilities.h"

typedef enum{
    NUS_CENTRAL_EVENT_PAIR_START,
    NUS_CENTRAL_EVENT_PAIR_TIMEOUT
}nus_central_event_t;
struct nus_central_event 
{
    nus_central_event_t event;
};

void ble_custom_nus_central_init(void);

int ble_custom_nus_central_start_find_pair_device(ble_custom_nus_fair_done_callback p_pair_done_callback);

void ble_custome_nus_central_stop_find_pair_device();

#endif