/*
 * Copyright (c) Bytech JSC 
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _HW_CODEC_H_
#define _HW_CODEC_H_
#include <math.h>
#include <stdint.h>

#define VPA        (5.0)
#define VDAC       (3.3)
#define MAX_GAIN   (20.0 * log10(VPA / VDAC))

/**
 * @brief Map of user volume to codec dac volume offset
 */
typedef float (*audio_codec_dac_vol_offset)(int volume);

typedef void *volume_handle_t;

/**
 * @brief Codec dac volume configurations
 */
typedef struct {
    float   max_dac_volume;  /*!< Codec support max volume */
    float   min_dac_volume;  /*!< Codec support min volume */
    float   board_pa_gain;   /*!< Board power amplifier gain */
    float   volume_accuracy; /*!< Codec dac volume accuracy(0.5 or 1) */
    int8_t  dac_vol_symbol;  /*!< Whether the dac volume is positively correlated with the register value */
    uint8_t zero_volume_reg; /*!< Codec register value for zero dac volume */
    uint8_t reg_value;       /*!< Record current dac volume register value */
    int32_t user_volume;     /*!< Record the user set volume */
    audio_codec_dac_vol_offset offset_conv_volume; /*!<  Convert user volume to dac volume offset */
}codec_dac_volume_config_t;


#include <stdint.h>

/**
 * @brief  Set volume on HW_CODEC
 *
 * @details Also unmutes the volume on HW_CODEC
 *
 * @param  set_val  Set the volume to a specific value.
 *                  This range of the value is between 0 to 128.
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_set(uint8_t set_val);

/**
 * @brief  Adjust volume on HW_CODEC
 *
 * @details Also unmute the volume on HW_CODEC
 *
 * @param  adjustment  The adjustment in dB, can be negative or positive.
 *			If the value 0 is used, the previous known value will be
 *			written, default value will be used if no previous value
 *			exists
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_adjust(int8_t adjustment);

/**
 * @brief Decrease output volume on HW_CODEC by 3 dB
 *
 * @details Also unmute the volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_decrease(void);

/**
 * @brief Increase output volume on HW_CODEC by 3 dB
 *
 * @details Also unmute the volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_increase(void);

/**
 * @brief  Mute volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_mute(void);

/**
 * @brief  Unmute volume on HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_volume_unmute(void);

/**
 * @brief Enable relevant settings in HW_CODEC to
 *        send and receive PCM data over I2S
 *
 * @note  FLL1 must be toggled after I2S has started to enable HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_default_conf_enable(void);

/**
 * @brief Reset HW_CODEC
 *
 * @note  This will first disable output, then do a soft reset
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_soft_reset(void);

/**
 * @brief Initialize HW_CODEC
 *
 * @return 0 if successful, error otherwise
 */
int hw_codec_init(void);


uint8_t audio_codec_get_dac_reg_value(volume_handle_t vol_handle, int volume);

float audio_codec_cal_dac_volume(volume_handle_t vol_handle);

volume_handle_t audio_codec_volume_init(codec_dac_volume_config_t *config);

void audio_codec_volume_deinit(volume_handle_t vol_handle);

float audio_codec_cal_dac_volume(volume_handle_t vol_handle);

#endif /* _HW_CODEC_H_ */
