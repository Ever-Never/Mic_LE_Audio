/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>

#include "macros_common.h"
#include "hw_codec.h"
#include "audio_hal.h"

#include "pair_ultilities.h"
#include <zephyr/logging/log.h>
#include "es8388.h"
LOG_MODULE_REGISTER(es8388, 3);
#define BOARD_PA_GAIN 0

#define ES8388_DAC_VOL_CFG_DEFAULT() {                      \
    .max_dac_volume = 0,                                    \
    .min_dac_volume = -96,                                  \
    .board_pa_gain = BOARD_PA_GAIN,                         \
    .volume_accuracy = 0.5,                                 \
    .dac_vol_symbol = -1,                                   \
    .zero_volume_reg = 0,                                   \
    .reg_value = 0,                                         \
    .user_volume = 0,                                       \
    .offset_conv_volume = NULL,                             \
}

static codec_dac_volume_config_t *dac_vol_handle;
static es_cfg_t const *p_es8388_cfg;
static es_cfg_t es8388_cfg =
{
    .addr = ES8388_ADDR
};
static const struct device *i2c_codec = DEVICE_DT_GET(DT_NODELABEL(i2c3));

int es8388_openbus(es_cfg_t const *p_cfg)
{
    if (p_es8388_cfg != NULL || p_cfg == NULL) {
		return -EINVAL;
	}
    p_es8388_cfg = p_cfg;
    return 0;
}
int es8388_closebus(es_cfg_t const *p_cfg)
{
	if (p_cfg == NULL || p_es8388_cfg != p_cfg || p_es8388_cfg == NULL) {
		return -EINVAL;
	}
	p_es8388_cfg = NULL;
	return 0;
}

static int es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    int ret;
    struct i2c_msg msgs[2];
    //uint8_t write_data[2] = {reg_add, 0};
    uint8_t read_data;
    if (!device_is_ready(i2c_codec) || p_es8388_cfg == NULL) {
		return -EIO;
	}
    memset(msgs, 0, sizeof(msgs));

    msgs[0].buf = &reg_add;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = (uint8_t*)&read_data;
    msgs[1].len = 1;
    msgs[1].flags = I2C_MSG_READ;

    ret = i2c_transfer(i2c_codec, msgs, 2 , ES8388_ADDR);
    *p_data = read_data;
    if(ret)
    {
        LOG_ERR("Failed to read i2c register\r\n");
        return ret;
    }
    return ret;
}


void es8388_read_all()
{
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
        //ets_printf("%x: %x\n", i, reg);
    }
}

int es8388_write_reg(uint8_t reg_add, uint8_t data)
{
    int ret;
    uint8_t write_data[2];
    struct i2c_msg msg;
    if (!device_is_ready(i2c_codec) || p_es8388_cfg == NULL) {
		return -EIO;
	}
    memset(&msg, 0, sizeof(msg));
    write_data[0] = reg_add;
    write_data[1] = data;
    msg.buf = write_data;
    msg.len = sizeof(write_data);
    msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;
    ret = i2c_transfer(i2c_codec, &msg, 1 , ES8388_ADDR);
    if(ret)
    {
        return ret;
    }
    return 0;
}

/**
 * @brief Configure ES8388 ADC and DAC volume. Basicly you can consider this as ADC and DAC gain
 *
 * @param mode:             set ADC or DAC or all
 * @param volume:           -96 ~ 0              for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 6); means set ADC volume -30.5db
 * @param dot:              whether include 0.5. for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 4); means set ADC volume -30db
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
static int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        LOG_WRN("Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es8388_write_reg(ES8388_ADCCONTROL8, volume);
        res |= es8388_write_reg(ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es8388_write_reg(ES8388_DACCONTROL5, volume);
        res |= es8388_write_reg(ES8388_DACCONTROL4, volume);
    }
    return res;
}


/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_start(es_module_t mode)
{
    int res = 0;
    res = es8388_openbus(&es8388_cfg);
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es8388_write_reg(ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es8388_write_reg(ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es8388_write_reg(ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es8388_write_reg(ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es8388_write_reg(ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
        res |= es8388_write_reg(ES8388_CHIPPOWER, 0xF0);   //start state machine
        // res |= es8388_write_reg(ES8388_CONTROL1, 0x16);
        // res |= es8388_write_reg(ES8388_CONTROL2, 0x50);
        res |= es8388_write_reg(ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
        res |= es8388_write_reg(ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
        res |= es8388_write_reg(ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= es8388_set_voice_mute(false);
        LOG_DBG("es8388_start default is mode:%d", mode);
    }

    return res;
}

/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_stop(es_module_t mode)
{
    int res = 0;
    if (mode == ES_MODULE_LINE) {
        res |= es8388_write_reg(ES8388_DACCONTROL21, 0x80); //enable dac
        res |= es8388_write_reg(ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
        res |= es8388_write_reg(ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
        res |= es8388_write_reg(ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
        return res;
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es8388_write_reg(ES8388_DACPOWER, 0x00);
        res |= es8388_set_voice_mute(true); //res |= Es8388SetAdcDacVolume(ES_MODULE_DAC, -96, 5);      // 0db
        //res |= es8388_write_reg(ES8388_DACPOWER, 0xC0);  //power down dac and line out
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        //res |= Es8388SetAdcDacVolume(ES_MODULE_ADC, -96, 5);      // 0db
        res |= es8388_write_reg(ES8388_ADCPOWER, 0xFF);  //power down adc and line in
    }
    if (mode == ES_MODULE_ADC_DAC) {
        res |= es8388_write_reg(ES8388_DACCONTROL21, 0x9C);  //disable mclk
//        res |= es8388_write_reg(ES8388_CONTROL1, 0x00);
//        res |= es8388_write_reg(ES8388_CONTROL2, 0x58);
//        res |= es8388_write_reg(ES8388_CHIPPOWER, 0xF3);  //stop state machine
    }

    return res;
}


/**
 * @brief Config I2s clock in MSATER mode
 *
 * @param cfg.sclkDiv:      generate SCLK by dividing MCLK in MSATER mode
 * @param cfg.lclkDiv:      generate LCLK by dividing MCLK in MSATER mode
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_i2s_config_clock(es_i2s_clock_t cfg)
{
    int res = 0;
    res |= es8388_write_reg(ES8388_MASTERMODE, cfg.sclk_div);
    res |= es8388_write_reg(ES8388_ADCCONTROL5, cfg.lclk_div);  //ADCFsMode,singel SPEED,RATIO=256
    res |= es8388_write_reg(ES8388_DACCONTROL2, cfg.lclk_div);  //ADCFsMode,singel SPEED,RATIO=256
    return res;
}

int es8388_deinit(void)
{
    int res = 0;
    res = es8388_write_reg(ES8388_CHIPPOWER, 0xFF);  //reset and stop es8388
    audio_codec_volume_deinit(dac_vol_handle);
    return res;
}

/**
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int es8388_init(audio_hal_codec_config_t *cfg)
{
    int res = 0;
    res |= es8388_write_reg(ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
    /* Chip Control and Power Management */
    res |= es8388_write_reg(ES8388_CONTROL2, 0x50);
    res |= es8388_write_reg(ES8388_CHIPPOWER, 0x00); //normal all and power up all

    // Disable the internal DLL to improve 8K sample rate
    res |= es8388_write_reg(0x35, 0xA0);
    res |= es8388_write_reg(0x37, 0xD0);
    res |= es8388_write_reg(0x39, 0xD0);

    res |= es8388_write_reg(ES8388_MASTERMODE, cfg->i2s_iface.mode); //CODEC IN I2S SLAVE MODE

    /* dac */
    res |= es8388_write_reg(ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    res |= es8388_write_reg(ES8388_CONTROL1, 0x12);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
//    res |= es8388_write_reg(ES8388_CONTROL2, 0);  //LPVrefBuf=0,Pdn_ana=0
    res |= es8388_write_reg(ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
    res |= es8388_write_reg(ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es8388_write_reg(ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es8388_write_reg(ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    res |= es8388_write_reg(ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    res |= es8388_write_reg(ES8388_DACCONTROL21, 0x80); // set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es8388_write_reg(ES8388_DACCONTROL23, 0x00); // vroi=0

    res |= es8388_write_reg(ES8388_DACCONTROL24, 0x1E); // Set L1 R1 L2 R2 volume. 0x00: -30dB, 0x1E: 0dB, 0x21: 3dB
    res |= es8388_write_reg(ES8388_DACCONTROL25, 0x1E);
    res |= es8388_write_reg(ES8388_DACCONTROL26, 0);
    res |= es8388_write_reg(ES8388_DACCONTROL27, 0);
    // res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);       // 0db
    int tmp = 0;
    if (AUDIO_HAL_DAC_OUTPUT_LINE2 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_ROUT1;
    } else if (AUDIO_HAL_DAC_OUTPUT_LINE1 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT2;
    } else {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    }
    res |= es8388_write_reg(ES8388_DACPOWER, tmp);  //0x3c Enable DAC and Enable Lout/Rout/1/2
    /* adc */
    res |= es8388_write_reg(ES8388_ADCPOWER, 0xFF);
    res |= es8388_write_reg(ES8388_ADCCONTROL1, 0xbb); // MIC Left and Right channel PGA gain
    tmp = 0;
    if (AUDIO_HAL_ADC_INPUT_LINE1 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT1_RINPUT1;
    } else if (AUDIO_HAL_ADC_INPUT_LINE2 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT2_RINPUT2;
    } else {
        tmp = ADC_INPUT_DIFFERENCE;
    }
    res |= es8388_write_reg(ES8388_ADCCONTROL2, tmp);  //0x00 LINSEL & RINSEL, LIN1/RIN1 as ADC Input; DSSEL,use one DS Reg11; DSR, LINPUT1-RINPUT1
    res |= es8388_write_reg(ES8388_ADCCONTROL3, 0x02);
    res |= es8388_write_reg(ES8388_ADCCONTROL4, 0x0c); // 16 Bits length and I2S serial audio data format
    res |= es8388_write_reg(ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
    //ALC for Microphone
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
    res |= es8388_write_reg(ES8388_ADCPOWER, 0x09);    // Power on ADC, enable LIN&RIN, power off MICBIAS, and set int1lp to low power mode
    
    /* es8388 PA gpio_config */

    es8388_pa_power(true);

    codec_dac_volume_config_t vol_cfg = ES8388_DAC_VOL_CFG_DEFAULT();
    dac_vol_handle = audio_codec_volume_init(&vol_cfg);
    LOG_INF("init,out:%02x, in:%02x", cfg->dac_output, cfg->adc_input);
    return res;
}

/**
 * @brief Configure ES8388 I2S format
 *
 * @param mode:           set ADC or DAC or all
 * @param bitPerSample:   see Es8388I2sFmt
 *
 * @return
 *     - (-1) Error
 *     - (0)  Success
 */
int es8388_config_fmt(es_module_t mode, es_i2s_fmt_t fmt)
{
    int res = 0;
    uint8_t reg = 0;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es8388_write_reg(ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es8388_write_reg(ES8388_DACCONTROL1, reg | (fmt << 1));
    }
    return res;
}

/**
 * @brief Set voice volume
 *
 * @note Register values. 0xC0: -96 dB, 0x64: -50 dB, 0x00: 0 dB
 * @note Accuracy of gain is 0.5 dB
 *
 * @param volume: voice volume (0~100)
 *
 * @return
 *     - 0
 *     - -1
 */
int es8388_set_voice_volume(int volume)
{
    int res = 0;
    uint8_t reg = 0;
    reg = audio_codec_get_dac_reg_value(dac_vol_handle, volume);
    res |= es8388_write_reg(ES8388_DACCONTROL5, reg);
    res |= es8388_write_reg(ES8388_DACCONTROL4, reg);
    LOG_DBG("Set volume:%.2d reg_value:0x%.2x dB:%.1f", dac_vol_handle->user_volume, reg,
            audio_codec_cal_dac_volume(dac_vol_handle));
    return res;
}

int es8388_get_voice_volume(int *volume)
{
    int res = 0;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL4, &reg);
    if (res == -1) {
        *volume = 0;
    } else {
        if (reg == dac_vol_handle->reg_value) {
            *volume = dac_vol_handle->user_volume;
        } else {
            *volume = 0;
            res = -1;
        }
    }
    LOG_DBG("Get volume:%.2d reg_value:0x%.2x", *volume, reg);
    return res;
}

/**
 * @brief Configure ES8388 data sample bits
 *
 * @param mode:             set ADC or DAC or all
 * @param bitPerSample:   see BitsLength
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_bits_per_sample(es_module_t mode, es_bits_length_t bits_length)
{
    int res = 0;
    uint8_t reg = 0;
    int bits = (int)bits_length;

    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es8388_write_reg(ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es8388_write_reg(ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}

/**
 * @brief Configure ES8388 DAC mute or not. Basically you can use this function to mute the output or unmute
 *
 * @param enable: enable or disable
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_voice_mute(bool enable)
{
    int res = 0;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es8388_write_reg(ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

int es8388_get_voice_mute(void)
{
    int res = 0;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    if (res == 0) {
        reg = (reg & 0x04) >> 2;
    }
    return res == 0 ? reg : res;
}

/**
 * @param gain: Config DAC Output
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_config_dac_output(es_dac_output_t output)
{
    int res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACPOWER, &reg);
    reg = reg & 0xc3;
    res |= es8388_write_reg(ES8388_DACPOWER, reg | output);
    return res;
}

/**
 * @param gain: Config ADC input
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_config_adc_input(es_adc_input_t input)
{
    int res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_ADCCONTROL2, &reg);
    reg = reg & 0x0f;
    res |= es8388_write_reg(ES8388_ADCCONTROL2, reg | input);
    return res;
}

/**
 * @param gain: see es_mic_gain_t
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int es8388_set_mic_gain(es8388_mic_gain_t gain)
{
    int res = 0;
    uint8_t gain_n = gain;
    //gain_n = (int)gain / 3;
    gain_n = (gain << 4) + gain;
    res = es8388_write_reg(ES8388_ADCCONTROL1, gain_n); //MIC PGA
    return res;
}
es8388_mic_gain_t es8388_get_mic_gain()
{
    int res = 0;
    uint8_t gain_n;
    res = es_read_reg(ES8388_ADCCONTROL1, &gain_n);
    gain_n = gain_n & 0x0F;
    gain_n = gain_n * 3;
    return gain_n;
}

int es8388_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    int res = 0;
    int es_mode_t = 0;
    switch (mode) {
        case AUDIO_HAL_CODEC_MODE_ENCODE:
            es_mode_t  = ES_MODULE_ADC;
            break;
        case AUDIO_HAL_CODEC_MODE_LINE_IN:
            es_mode_t  = ES_MODULE_LINE;
            break;
        case AUDIO_HAL_CODEC_MODE_DECODE:
            es_mode_t  = ES_MODULE_DAC;
            break;
        case AUDIO_HAL_CODEC_MODE_BOTH:
            es_mode_t  = ES_MODULE_ADC_DAC;
            break;
        default:
            es_mode_t = ES_MODULE_DAC;
            LOG_WRN("Codec mode not support, default is decode mode");
            break;
    }
    if (AUDIO_HAL_CTRL_STOP == ctrl_state) {
        res = es8388_stop(es_mode_t);
    } else {
        res = es8388_start(es_mode_t);
        LOG_DBG("start default is decode mode:%d", es_mode_t);
    }
    return res;
}

int es8388_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    int res = 0;
    int tmp = 0;
    res |= es8388_config_fmt(ES_MODULE_ADC_DAC, iface->fmt);
    if (iface->bits == AUDIO_HAL_BIT_LENGTH_16BITS) {
        tmp = BIT_LENGTH_16BITS;
    } else if (iface->bits == AUDIO_HAL_BIT_LENGTH_24BITS) {
        tmp = BIT_LENGTH_24BITS;
    } else {
        tmp = BIT_LENGTH_32BITS;
    }
    res |= es8388_set_bits_per_sample(ES_MODULE_ADC_DAC, tmp);
    return res;
}

void es8388_pa_power(bool enable)
{
    // if (enable) {
    //     gpio_set_level(get_pa_enable_gpio(), 1);
    // } else {
    //     gpio_set_level(get_pa_enable_gpio(), 0);
    // }
    return;
}