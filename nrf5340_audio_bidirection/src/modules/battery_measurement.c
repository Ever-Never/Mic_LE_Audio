/*
 * Copyright (c) BYTECH JSC
 *
 */

#include "board_version.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <nrfx_saadc.h>

#include "board.h"
#include "macros_common.h"

#include <zephyr/logging/log.h>
#include "ctrl_events.h"
LOG_MODULE_REGISTER(battery_measurement, 5);

static const struct device *adc_battery;
//static const struct gpio_dt_spec battery_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bat_measure), gpios);

#define ADC_1ST_CHANNEL_ID 0
#define ADC_RESOLUTION_BITS 12
/* We allow the ADC register value to deviate by N points in either direction */
#define ADC_ACQ_TIME_US 40
#define VOLTAGE_STABILIZE_TIME_US 5




static int16_t sample_buffer;
static const struct adc_channel_cfg m_channel_cfg = {
	.gain = ADC_GAIN_1_4,
	.reference = ADC_REF_VDD_1_4,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, ADC_ACQ_TIME_US),
	.channel_id = ADC_1ST_CHANNEL_ID,
	.differential = 0,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = NRF_SAADC_INPUT_AIN5,
#endif /* defined(CONFIG_ADC_CONFIGURABLE_INPUTS) */
};
static const struct adc_sequence adc_sequence = {
	.channels = BIT(ADC_1ST_CHANNEL_ID),
	.buffer = (void *)&sample_buffer,
	.buffer_size = sizeof(sample_buffer),
	.resolution = ADC_RESOLUTION_BITS,
	.oversampling = NRF_SAADC_OVERSAMPLE_256X,
};


#define BATTERY_ADC_RESOLUTION  4095 // 2^12 -1
#define BATTERY_ADC_GAIN        4
#define BATTERY_REFERENCE_VOLTAGE 825
#define BATTERY_VOLTAGE_DIVIDER_RATIO   2
static bool m_initialized = false;


/*<Forward declaration>*/
static void adc_tick(struct k_timer *timer);
K_TIMER_DEFINE(adc_timer_tick, adc_tick, NULL);
int battery_measurement_init()
{
    int ret = 0;
    if(m_initialized == true)
    {
        LOG_WRN("Battery measurement already initialized\r\n");
        return 0;
    }
    adc_battery = DEVICE_DT_GET(DT_NODELABEL(adc));
	if (!device_is_ready(adc_battery)) {
		LOG_ERR("ADC not ready");
		return -ENXIO;
	}
    ret = adc_channel_setup(adc_battery, &m_channel_cfg);
	if (ret) {
		return ret;
	}
    k_timer_start(&adc_timer_tick, K_SECONDS(5), K_SECONDS(5));
	m_initialized = true;
    LOG_INF("Battery measurement init\r\n");
	return 0;
}


int battery_raw_data_get(uint16_t *bat_vol)
{
    int ret = 0;
    uint16_t battery_voltage;
    ret = adc_read(adc_battery, &adc_sequence);
    if(ret != 0)
    {
        LOG_ERR("Failed to read adc battery err:%d\r\n", ret);
    }
    battery_voltage = ((sample_buffer * BATTERY_REFERENCE_VOLTAGE)/ BATTERY_ADC_RESOLUTION) * BATTERY_ADC_GAIN; 
    battery_voltage = battery_voltage * BATTERY_VOLTAGE_DIVIDER_RATIO;
    LOG_INF("Battery voltage: %u - reg val: %u\r\n", battery_voltage, sample_buffer);
    return ret;
}

static void adc_tick(struct k_timer *timer)
{
    /*Put battery timeout tick to event queue*/
	struct event_t battery;
	battery.battery_activity.event = BATTERY_EVENT_MEASURE;
	battery.event_source = EVT_SRC_ADC;
	if(ctrl_events_queue_empty())
	{
	    int ret = ctrl_events_put(&battery);
		ERR_CHK(ret);
	}
	
}