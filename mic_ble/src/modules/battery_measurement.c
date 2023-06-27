/*
 * Copyright (c) BYTECH JSC
 *
 */

// #include "board_version.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>
#include <nrfx_saadc.h>

#include "board.h"
#include "macros_common.h"

#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "battery_measurement.h"
//#include "ctrl_events.h"
LOG_MODULE_REGISTER(battery_measurement, 5);

static const struct device *adc_battery;
//static const struct gpio_dt_spec battery_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bat_measure), gpios);

#define ADC_1ST_CHANNEL_ID 0
#define ADC_RESOLUTION_BITS 12
/* We allow the ADC register value to deviate by N points in either direction */
#define ADC_ACQ_TIME_US 40
#define VOLTAGE_STABILIZE_TIME_US 5

#define BATTERY_VOLTAGE_MAX_MV	4200
#define BATTERY_VOLTAGE_MIN_MV	2700

/*Create task to measure battery data*/

#define CONFIG_BATTERY_MEASURE_PUBLISH_STACK_SIZE	300
#define CONFIG_BATTERY_MEASURE_PUBLISH_THREAD_PRIO 5


static void battery_measurement_publish(void);

ZBUS_CHAN_DEFINE(battery_measure_channel, struct battery_event, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));
K_THREAD_DEFINE(battery_measure_publish_thread_id, CONFIG_BATTERY_MEASURE_PUBLISH_STACK_SIZE, battery_measurement_publish,
 		NULL, NULL, NULL, K_PRIO_PREEMPT(CONFIG_BATTERY_MEASURE_PUBLISH_THREAD_PRIO), 0, 0);

K_MSGQ_DEFINE(battery_to_queue, sizeof(struct battery_event), 1, 4);


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
    //LOG_INF("Battery voltage: %u - reg val: %u - BatteryPercent:%u%%\r\n", battery_voltage, (uint16_t)sample_buffer);
    return ret;
}



static void battery_measurement_publish(void)
{
	int ret;
	struct battery_event msg;

	while (1) {
		k_msgq_get(&battery_to_queue, &msg, K_FOREVER);

		ret = zbus_chan_pub(&battery_measure_channel, &msg, K_NO_WAIT);
		if (ret) {
			LOG_ERR("Failed to publish button msg, ret: %d", ret);
		}
	}
}
static void adc_tick(struct k_timer *timer)
{
	int ret = 0;
	struct battery_event event;
	event.event = BATTERY_EVENT_MEASURE;
	ret = k_msgq_put(&battery_to_queue, (void *)&event, K_NO_WAIT);
	if(ret == -EAGAIN)
	{
		LOG_WRN("Btn msg queue full");
	}
}