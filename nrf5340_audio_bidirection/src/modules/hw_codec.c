/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "hw_codec.h"

#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>

#include "macros_common.h"
#include "cs47l63.h"
#include "cs47l63_spec.h"
#include "cs47l63_reg_conf.h"
#include "cs47l63_comm.h"
#include "es8311.h"
#include "es8388.h"
#include "button_handler.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hw_codec, CONFIG_MODULE_HW_CODEC_LOG_LEVEL);

#define VOLUME_ADJUST_STEP_DB 3
#define HW_CODEC_SELECT_DELAY_MS 2
#define AUDIO_HAL_VOL_DEFAULT 30
#define BASE_10 10

#define VOLUME_DECREMENT_STEP	4
#define VOLUME_INCREMENT_STEP	4
static codec_dac_volume_config_t m_dac_volume_handle;


#define AUDIO_CODEC_DEFAULT_CONFIG(){                   \
        .adc_input  = AUDIO_HAL_ADC_INPUT_LINE1,        \
        .dac_output = AUDIO_HAL_DAC_OUTPUT_LINE1,         \
        .codec_mode = AUDIO_HAL_CODEC_MODE_BOTH,        \
        .i2s_iface = {                                  \
            .mode = AUDIO_HAL_MODE_SLAVE,               \
            .fmt = AUDIO_HAL_I2S_NORMAL,                \
            .samples = AUDIO_HAL_48K_SAMPLES,           \
            .bits = AUDIO_HAL_BIT_LENGTH_16BITS,        \
        },                                              \
};

/*
 * User can customize the volume setting by modifying the mapping table and adjust the volume step according to
 * the speaker playback system, and the other volume levels shift the value accordingly.
 * Integers are used instead of floating-point variables to reduce storage space. -80 means -40 dB, 0 means 0 dB.
 */
static const int8_t dac_volume_offset[] = {
    -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80,
    -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60,
    -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40,
    -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20,
    -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9,  -8,  -7,  -6,  -5,  -4,  -3,  -2,  -1,   0
};

static inline uint8_t audio_codec_calculate_reg(volume_handle_t vol_handle, float dac_volume)
{
    codec_dac_volume_config_t *handle = (codec_dac_volume_config_t *) vol_handle;
    uint8_t reg = (uint8_t) (dac_volume / (handle->dac_vol_symbol * handle->volume_accuracy) + handle->zero_volume_reg);
    return reg;
}
/**
 * @brief Get DAC volume offset from user set volume, you can use an array or function to finish this map
 *
 * @note The max DAC volume is 0 dB when the user volume is 100. 0 dB means there is no attenuation of the sound source,
 *       and it is the original sound source. It can not exceed 0 dB. Otherwise, there is a risk of clipping noise.
 * @note For better audio dynamic range, we'd better use 0dB full scale digital gain and lower analog gain.
 * @note DAC volume offset is positively correlated with the user volume.
 *
 * @param volume User set volume (1-100)
 *
 * @return
 *     - Codec DAC volume offset. The max value must be 0 dB.
 */
static inline float codec_get_dac_volume_offset(int volume)
{
    float offset = dac_volume_offset[volume - 1] / 2.0;
    return offset;
}

#define MIC_SHUTDOWN_NODE	DT_ALIAS(mic)
static const struct gpio_dt_spec mic_shutdown = GPIO_DT_SPEC_GET(MIC_SHUTDOWN_NODE, gpios);

static const int es8311_mic_gain[] =
{
    ES8311_MIC_GAIN_0DB, 
    ES8311_MIC_GAIN_6DB,
    ES8311_MIC_GAIN_12DB,
    ES8311_MIC_GAIN_18DB,
    ES8311_MIC_GAIN_24DB,
    ES8311_MIC_GAIN_30DB,
    ES8311_MIC_GAIN_36DB,
    ES8311_MIC_GAIN_42DB
};
static const int es8388_mic_gain[] = 
{
    MIC_GAIN_0DB,
    MIC_GAIN_3DB,
    MIC_GAIN_6DB,
    MIC_GAIN_9DB,
    MIC_GAIN_12DB,
    MIC_GAIN_15DB,
    MIC_GAIN_18DB,
    MIC_GAIN_21DB,
    MIC_GAIN_24DB
};
#define ES8311_MIC_GAIN_SIZE		(sizeof(es8311_mic_gain)/sizeof(es8311_mic_gain[0]))
#define ES8388_MIC_GAIN_SIZE		(sizeof(es8388_mic_gain)/sizeof(es8388_mic_gain[0]))
static int get_codec_mic_volume(uint8_t volume_percent)
{
	int ret_mic_gain = 0;
	
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	volume_percent = volume_percent / ARRAY_SIZE(es8311_mic_gain);
	if(volume_percent >= ES8311_MIC_GAIN_SIZE - 1)
	{
		ret_mic_gain = es8311_mic_gain[ES8311_MIC_GAIN_SIZE - 1];
	}
	else if(volume_percent <= 0)
	{
		ret_mic_gain = es8311_mic_gain[0];
	}
	else
	{
		ret_mic_gain = es8311_mic_gain[volume_percent];
	}
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	volume_percent = volume_percent / ARRAY_SIZE(es8388_mic_gain);
	if(volume_percent >= ARRAY_SIZE(es8388_mic_gain) - 1)
	{
		ret_mic_gain = es8388_mic_gain[ARRAY_SIZE(es8388_mic_gain) - 1];
	}
	else if(volume_percent <= 0)
	{
		ret_mic_gain = es8388_mic_gain[0];
	}
	else
	{
		ret_mic_gain = es8388_mic_gain[volume_percent];
	}
#endif
	return ret_mic_gain;
}
/**@brief Select the on-board HW codec
 */
static int hw_codec_on_board_set(void)
{
	k_msleep(HW_CODEC_SELECT_DELAY_MS);
	return 0;
}

int hw_codec_volume_set(uint8_t set_val)
{
	int ret = 0;
#if(CONFIG_AUDIO_DEV == HEADSET)
	for(uint8_t retry = 0; retry < 10; retry++)
	{
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
		ret = es8311_codec_set_voice_volume(set_val);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
		ret = es8388_set_voice_volume(set_val);
#endif
		if(ret == 0)
		{
			return ret;
		}
	}
#elif(CONFIG_AUDIO_DEV == GATEWAY)
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	es8311_mic_gain_t gain = get_codec_mic_volume(set_val);
	ret = es8311_set_mic_gain(gain);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	es8388_mic_gain_t gain = get_codec_mic_volume(set_val);
	ret = es8388_set_mic_gain(set_val);
#endif
	if(ret)
	{
		return -1;
	}
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	es8311_mic_gain_t read_back = es8311_get_mic_gain();
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	//es_mic_gain_t read_back = es8388__mic
	es8388_mic_gain_t read_back = es8388_get_mic_gain();
#endif
	if(read_back != gain)
	{
		LOG_ERR("Set codec volume failed:%d\r\n", ret);
		return -1;
	}
#endif
	return ret;
}

int hw_codec_volume_adjust(int8_t adjustment_db)
{

	return 0;
}

int hw_codec_volume_decrease(void)
{
	int ret;
#if(CONFIG_AUDIO_DEV == HEADSET)
	int l_current_volume = 0;
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	ret = es8311_codec_get_voice_volume(&l_current_volume);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	ret = es8388_get_voice_volume(&l_current_volume);
#endif
	if(ret == 0)
	{
		l_current_volume = l_current_volume - VOLUME_DECREMENT_STEP;
	}
	else
	{
		LOG_ERR("Failed to increase volume :%d\r\n", ret);
		return ret;
	}
	if(l_current_volume <=  0)
	{
		l_current_volume = 0;
	}
	for(uint8_t retry = 0; retry < 10; retry++)
	{
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
		ret = es8311_codec_set_voice_volume(l_current_volume);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
		ret = es8388_set_voice_volume(l_current_volume);
#endif
		if(ret == 0)
		{
			break;
		}
	}
#else
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	es8311_mic_gain_t current_gain = es8311_get_mic_gain();
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	es8388_mic_gain_t current_gain = es8388_get_mic_gain();
#endif
	current_gain--;
	if(current_gain <= 0)
	{
		current_gain = 0;
	}
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	ret = es8311_set_mic_gain(current_gain);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	ret = es8388_set_mic_gain(current_gain);
#endif
	LOG_INF("Decrease mic gain to level:%u", current_gain);
	if(ret)
	{
		LOG_ERR("Fail to set mic gain");
		return ret;
	}
#endif
	return 0;
}

int hw_codec_volume_increase(void)
{
	int ret = 0;
#if(CONFIG_AUDIO_DEV == HEADSET)
	int l_current_volume = 0;
	#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
		ret = es8311_codec_get_voice_volume(&l_current_volume);
	#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
		ret = es8388_get_voice_volume(&l_current_volume);
	#endif
	if(ret == 0)
	{
		l_current_volume = l_current_volume + VOLUME_INCREMENT_STEP;
	}
	else
	{
		LOG_ERR("Failed to increase volume :%d\r\n", ret);
		return ret;
	}
	if(l_current_volume >= 100)
	{
		l_current_volume = 100;
	}
	for(uint8_t retry = 0; retry < 10; retry++)
	{
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
		ret = es8311_codec_set_voice_volume(l_current_volume);

#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
		ret = es8388_set_voice_volume(l_current_volume);

#endif
		if(ret == 0)
		{
			break;
		}
	}
#else
	#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
		es8311_mic_gain_t current_gain = es8311_get_mic_gain();
	#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
		es8388_mic_gain_t current_gain = es8388_get_mic_gain();
	#endif
	current_gain++;
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	if(current_gain >= ES8311_MIC_GAIN_42DB)
	{
		current_gain = ES8311_MIC_GAIN_42DB;
	}

#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	
	if(current_gain >= MIC_GAIN_24DB)
	{
		current_gain = MIC_GAIN_24DB;
	}
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	ret = es8311_set_mic_gain(current_gain);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	ret = es8388_set_mic_gain(current_gain);
#endif
	LOG_INF("Increase mic gain to level:%u", current_gain);
#endif
	if(ret)
	{
		LOG_ERR("Fail to set mic gain");
		return ret;
	}
#endif
	return ret;
}

int hw_codec_volume_mute(void)
{
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	es8311_mute(1);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	es8388_set_voice_mute(1);
#endif
	return 0;
}

int hw_codec_volume_unmute(void)
{
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	es8311_mute(0);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	es8388_set_voice_mute(0);
#endif
	return 0;
}

int hw_codec_default_conf_enable(void)
{
	return 0;
}

int hw_codec_soft_reset(void)
{
	return 0;
}

int hw_codec_init(void)
{
	int ret;

	/* Set to internal/on board codec */
	ret = hw_codec_on_board_set();
	if (ret) {
		return ret;
	}
	/*turn on the microphone preamp*/
	if(!device_is_ready(mic_shutdown.port))
	{
		ret = -1;
		LOG_ERR("Mic shutdown port failed\r\n");
		return ret;
	}
	ret = gpio_pin_configure_dt(&mic_shutdown, GPIO_OUTPUT_LOW);
	LOG_INF("Mic shut down state: %d\r\n", gpio_pin_get_dt(&mic_shutdown));
	audio_hal_codec_config_t audio_config = AUDIO_CODEC_DEFAULT_CONFIG(); 
#if(CONFIG_AUDIO_DEV == GATEWAY)
	audio_config.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
#else(CONFIG_AUDIO_DEV == HEADSET)
	audio_config.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
#endif
	//audio_config.adc_input = AUDIO_HAL_ADC_INPUT_LINE2;
	/*Config the codec*/
#if(CONFIG_AUDIO_HARDWARE_CODEC == 1)
	ret = es8311_init(&audio_config);
	if(ret){
		return ret;
	}
	// /*set default volume*/
	es8311_codec_config_i2s(audio_config.codec_mode, &audio_config.i2s_iface);
	es8311_codec_ctrl_state(AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
#elif(CONFIG_AUDIO_HARDWARE_CODEC == 2)
	ret = es8388_init(&audio_config);
	if(ret){
		return ret;
	}
	// /*set default volume*/
	es8388_config_i2s(audio_config.codec_mode, &audio_config.i2s_iface);
	es8388_ctrl_state(audio_config.codec_mode, AUDIO_HAL_CTRL_START);

#endif
#if(CONFIG_AUDIO_DEV == HEADSET)
	hw_codec_volume_set(100); /*Set out put volume if device is slave*/
#elif(CONFIG_AUDIO_DEV == GATEWAY)
	hw_codec_volume_set(100); /*Set input volume if device is master*/
#endif
	if(app_button_read_detect_mic())
	{
		hw_codec_set_input(1);
	}
	else
	{
		hw_codec_set_input(0);
	}
	return 0;
}
/*Only for es8388*/
int hw_codec_set_input(bool line_true)
{
	audio_hal_adc_input_t input;
	if(line_true) 
	{
		LOG_INF("Using line in -> Change mic MicAmp to 6dbm");
		input = AUDIO_HAL_ADC_INPUT_LINE2;
		/*
			@brief: If user choose to use line in -> Input voltage will much more higher than using on board mic
					-> Unexpecting 
		*/
		es8388_set_mic_gain(MIC_GAIN_6DB);

	}
	else 
	{
		input = AUDIO_HAL_ADC_INPUT_LINE1;
	}
	return es8388_select_input(input);
}
static int cmd_input(const struct shell *shell, size_t argc, char **argv)
{
	int ret;
	uint8_t idx;

	enum hw_codec_input {
		LINE_IN,
		PDM_MIC,
		NUM_INPUTS,
	};

	if (argc != 2) {
		shell_error(shell, "Only one argument required, provided: %d", argc);
		return -EINVAL;
	}

	if ((CONFIG_AUDIO_DEV == GATEWAY) && IS_ENABLED(CONFIG_AUDIO_SOURCE_USB)) {
		shell_error(shell, "Can't select PDM mic if audio source is USB");
		return -EINVAL;
	}

	if ((CONFIG_AUDIO_DEV == HEADSET) && !IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL)) {
		shell_error(shell, "Can't select input if headset is not in bidirectional stream");
		return -EINVAL;
	}

	if (!isdigit((int)argv[1][0])) {
		shell_error(shell, "Supplied argument is not numeric");
		return -EINVAL;
	}

	idx = strtoul(argv[1], NULL, BASE_10);

	switch (idx) {
	case LINE_IN: {
		if (CONFIG_AUDIO_DEV == HEADSET) {
			if (ret) {
				shell_error(shell, "Failed to enable LINE-IN");
				return ret;
			}
		}


		shell_print(shell, "Selected LINE-IN as input");
		break;
	}
	case PDM_MIC: {
		if (CONFIG_AUDIO_DEV == GATEWAY) {
		}
		shell_print(shell, "Selected PDM mic as input");
		break;
	}
	default:
		shell_error(shell, "Invalid input");
		return -EINVAL;
	}

	return 0;
}
volume_handle_t audio_codec_volume_init(codec_dac_volume_config_t *config)
{
	if(config == NULL)
	{
		return NULL; // Invalid argument
	}
    memcpy(&m_dac_volume_handle, config, sizeof(codec_dac_volume_config_t)); // try to use static memory, 
    if (!m_dac_volume_handle.offset_conv_volume) {
        m_dac_volume_handle.offset_conv_volume = codec_get_dac_volume_offset;
    }
    return (volume_handle_t)&m_dac_volume_handle;
}

/**
 * @brief Take zero dac_volume as the origin and calculate the volume offset according to the register value
 */
float audio_codec_cal_dac_volume(volume_handle_t vol_handle)
{
    codec_dac_volume_config_t *handle = (codec_dac_volume_config_t *)vol_handle;
    float dac_volume = handle->dac_vol_symbol * handle->volume_accuracy * (handle->reg_value - handle->zero_volume_reg);
    return dac_volume;
}

uint8_t audio_codec_get_dac_reg_value(volume_handle_t vol_handle, int volume)
{
    float dac_volume = 0;
    int user_volume = volume;
    codec_dac_volume_config_t *handle = (codec_dac_volume_config_t *)vol_handle;

    if (user_volume < 0) {
        user_volume = 0;
    } else if (user_volume > 100) {
        user_volume = 100;
    }
    if (user_volume == 0) {
        dac_volume = handle->min_dac_volume; // Make sure the speaker voice is near silent
    } else {
        /*
         * For better audio performance, at the max volume, we need to ensure:
         * Audio Process Gain + Codec DAC Volume + PA Gain <= MAX_GAIN.
         * The PA Gain and Audio Process Gain are known when the board design is fixed, so
         * max Codec DAC Volume = MAX_GAIN - PA Gain - Audio Process Gain，then
         * the volume mapping table shift accordingly.
         */
        dac_volume = handle->offset_conv_volume(user_volume) + MAX_GAIN - handle->board_pa_gain;
        dac_volume = dac_volume < handle->max_dac_volume ? dac_volume : handle->max_dac_volume;
    }
    handle->reg_value = audio_codec_calculate_reg(handle, dac_volume);
    handle->user_volume = user_volume;
    return handle->reg_value;
}


void audio_codec_volume_deinit(volume_handle_t vol_handle)
{
    if (vol_handle) {
		vol_handle = NULL;
		memset(&m_dac_volume_handle, 0, sizeof(m_dac_volume_handle)); //i try to avoid dynamic memory allocation :D, not safe for me lmao
    }
}


SHELL_STATIC_SUBCMD_SET_CREATE(hw_codec_cmd,
			       SHELL_COND_CMD(CONFIG_SHELL, input, NULL,
					      " Select input\n\t0: LINE_IN\n\t\t1: PDM_MIC",
					      cmd_input),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(hw_codec, &hw_codec_cmd, "Change settings on HW codec", NULL);
