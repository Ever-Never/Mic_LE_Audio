#ifndef _BATTERY_MEASUREMENT_
#define _BATTERY_MEASUREMENT_
#include <stdint.h>

typedef enum{
    BATTERY_EVENT_START,
    BATTERY_EVENT_MEASURE
}battery_event_t;
struct battery_event 
{
    battery_event_t event;
};

int battery_measurement_init();

int battery_raw_data_get(uint16_t *bat_vol);

#endif