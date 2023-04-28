#include "es8311.h"
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>

#include "macros_common.h"
#include "hw_codec.h"

#include "pair_ultilities.h"
#include "audio_hal.h"

/* ES8311 address
 * 0x32:CE=1;0x30:CE=0
 */
#define ES8311_ADDR         0x18

/*
 * to define the clock soure of MCLK
 */
#define FROM_MCLK_PIN       0
#define FROM_SCLK_PIN       1

/*
 * to define whether to reverse the clock
 */
#define INVERT_MCLK         0 // do not invert
#define INVERT_SCLK         0

#define IS_DMIC             0 // Is it a digital microphone

#define MCLK_DIV_FRE        256

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(es8311, 3);
//LOG_MODULE_REGISTER(es8388, CONFIG_ES8388_LOG_LEVEL);

/*
 * Clock coefficient structer
 */
struct _coeff_div {
    uint32_t mclk;        /* mclk frequency */
    uint32_t rate;        /* sample rate */
    uint8_t pre_div;      /* the pre divider with range from 1 to 8 */
    uint8_t pre_multi;    /* the pre multiplier with x1, x2, x4 and x8 selection */
    uint8_t adc_div;      /* adcclk divider */
    uint8_t dac_div;      /* dacclk divider */
    uint8_t fs_mode;      /* double speed or single speed, =0, ss, =1, ds */
    uint8_t lrck_h;       /* adclrck divider and daclrck divider */
    uint8_t lrck_l;
    uint8_t bclk_div;     /* sclk divider */
    uint8_t adc_osr;      /* adc osr */
    uint8_t dac_osr;      /* dac osr */
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
    //mclk     rate   pre_div  mult  adc_div dac_div fs_mode lrch  lrcl  bckdiv osr
    /* 8k */
    {12288000, 8000 , 0x06, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {18432000, 8000 , 0x03, 0x02, 0x03, 0x03, 0x00, 0x05, 0xff, 0x18, 0x10, 0x20},
    {16384000, 8000 , 0x08, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {8192000 , 8000 , 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {6144000 , 8000 , 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {4096000 , 8000 , 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {3072000 , 8000 , 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {2048000 , 8000 , 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {1536000 , 8000 , 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {1024000 , 8000 , 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},

    /* 11.025k */
    {11289600, 11025, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {5644800 , 11025, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {2822400 , 11025, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {1411200 , 11025, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},

    /* 12k */
    {12288000, 12000, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {6144000 , 12000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {3072000 , 12000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {1536000 , 12000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},

    /* 16k */
    {12288000, 16000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {18432000, 16000, 0x03, 0x02, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x20},
    {16384000, 16000, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {8192000 , 16000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {6144000 , 16000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {4096000 , 16000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {3072000 , 16000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {2048000 , 16000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {1536000 , 16000, 0x03, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},
    {1024000 , 16000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20},

    /* 22.05k */
    {11289600, 22050, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {5644800 , 22050, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2822400 , 22050, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1411200 , 22050, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 24k */
    {12288000, 24000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 24000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000 , 24000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000 , 24000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000 , 24000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 32k */
    {12288000, 32000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 32000, 0x03, 0x04, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10},
    {16384000, 32000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {8192000 , 32000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000 , 32000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {4096000 , 32000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000 , 32000, 0x03, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2048000 , 32000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000 , 32000, 0x03, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},
    {1024000 , 32000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 44.1k */
    {11289600, 44100, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {5644800 , 44100, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2822400 , 44100, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1411200 , 44100, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 48k */
    {12288000, 48000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 48000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000 , 48000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000 , 48000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000 , 48000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 64k */
    {12288000, 64000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 64000, 0x03, 0x04, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10},
    {16384000, 64000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {8192000 , 64000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000 , 64000, 0x01, 0x04, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10},
    {4096000 , 64000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000 , 64000, 0x01, 0x08, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10},
    {2048000 , 64000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000 , 64000, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0xbf, 0x03, 0x18, 0x18},
    {1024000 , 64000, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},

    /* 88.2k */
    {11289600, 88200, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {5644800 , 88200, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2822400 , 88200, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1411200 , 88200, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},

    /* 96k */
    {12288000, 96000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 96000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000 , 96000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000 , 96000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000 , 96000, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},
};

static es_cfg_t es8311_cfg =
{
    .addr = ES8311_ADDR
};


static codec_dac_volume_config_t *dac_vol_handle;

#define ES8311_DAC_VOL_CFG_DEFAULT() {                      \
    .max_dac_volume = 32,                                   \
    .min_dac_volume = -95.5,                                \
    .board_pa_gain = 0,                         \
    .volume_accuracy = 0.5,                                 \
    .dac_vol_symbol = 1,                                    \
    .zero_volume_reg = 0xBF,                                \
    .reg_value = 0,                                         \
    .user_volume = 0,                                       \
    .offset_conv_volume = NULL,                             \
}


static es_cfg_t const *p_es8311_cfg;
static const struct device *i2c_codec = DEVICE_DT_GET(DT_NODELABEL(i2c3));
int8_t get_es8311_mclk_src(void);
static int get_coeff(uint32_t mclk, uint32_t rate);
static int es8311_write_reg(uint8_t reg_add, uint8_t data)
{
    //return i2c_bus_write_bytes(i2c_handle, slave_addr, &reg_add, sizeof(reg_add), &data, sizeof(data));
    int ret;
    uint8_t write_data[2];
    struct i2c_msg msg;
    if (!device_is_ready(i2c_codec) || p_es8311_cfg == NULL) {
		return -EIO;
	}
    memset(&msg, 0, sizeof(msg));
    write_data[0] = reg_add;
    write_data[1] = data;
    msg.buf = write_data;
    msg.len = sizeof(write_data);
    msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;
    ret = i2c_transfer(i2c_codec, &msg, 1 , ES8311_ADDR);
    if(ret)
    {
        return ret;
    }
    return 0;
}
static int es8311_read_reg(uint8_t reg_add)
{
    int ret;
    struct i2c_msg msgs[2];
    //uint8_t write_data[2] = {reg_add, 0};
    uint8_t read_data;
    if (!device_is_ready(i2c_codec) || p_es8311_cfg == NULL) {
		return -EIO;
	}
    memset(msgs, 0, sizeof(msgs));

    msgs[0].buf = &reg_add;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = (uint8_t*)&read_data;
    msgs[1].len = 1;
    msgs[1].flags = I2C_MSG_READ;

    ret = i2c_transfer(i2c_codec, msgs, 2 , ES8311_ADDR);
    if(ret)
    {
        LOG_ERR("Failed to read i2c register\r\n");
        return ret;
    }
    return read_data;
}

int es8311_openbus(es_cfg_t const *p_cfg)
{
    if (p_es8311_cfg != NULL || p_cfg == NULL) {
		return -EINVAL;
	}
    p_es8311_cfg = p_cfg;
    return 0;
}
int es8311_closebus(es_cfg_t const *p_cfg)
{
	if (p_cfg == NULL || p_es8311_cfg != p_cfg || p_es8311_cfg == NULL) {
		return -EINVAL;
	}
	p_es8311_cfg = NULL;
	return 0;
}

/*
* set es8311 dac mute or not
* if mute = 0, dac un-mute
* if mute = 1, dac mute
*/

/*
* look for the coefficient in coeff_div[] table
*/
static int get_coeff(uint32_t mclk, uint32_t rate)
{
    for (int i = 0; i < (sizeof(coeff_div) / sizeof(coeff_div[0])); i++) {
        if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
            return i;
    }
    return -1;
}
void es8311_mute(int mute)
{
    uint8_t regv;
    LOG_INF("Enter into es8311_mute(), mute = %d\n", mute);
    regv = es8311_read_reg(ES8311_DAC_REG31) & 0x9f;
    if (mute) {
        es8311_write_reg(ES8311_SYSTEM_REG12, 0x02);
        es8311_write_reg(ES8311_DAC_REG31, regv | 0x60);
        es8311_write_reg(ES8311_DAC_REG32, 0x00);
        es8311_write_reg(ES8311_DAC_REG37, 0x08);
    } else {
        es8311_write_reg(ES8311_DAC_REG31, regv);
        es8311_write_reg(ES8311_SYSTEM_REG12, 0x00);
    }
}
/*
* set es8311 into suspend mode
*/
static void es8311_suspend(void)
{
    LOG_INF("Enter into es8311_suspend()");
    es8311_write_reg(ES8311_DAC_REG32, 0x00);
    es8311_write_reg(ES8311_ADC_REG17, 0x00);
    es8311_write_reg(ES8311_SYSTEM_REG0E, 0xFF);
    es8311_write_reg(ES8311_SYSTEM_REG12, 0x02);
    es8311_write_reg(ES8311_SYSTEM_REG14, 0x00);
    es8311_write_reg(ES8311_SYSTEM_REG0D, 0xFA);
    es8311_write_reg(ES8311_ADC_REG15, 0x00);
    es8311_write_reg(ES8311_DAC_REG37, 0x08);
    es8311_write_reg(ES8311_GP_REG45, 0x01);
}

/*
* enable pa power
*/
void es8311_pa_power(bool enable)
{
    LOG_DBG("No PA support\r\n");
    // if (enable) {
    //     gpio_set_level(get_pa_enable_gpio(), 1);
    // } else {
    //     gpio_set_level(get_pa_enable_gpio(), 0);
    // }
}

int es8311_init(audio_hal_codec_config_t *codec_cfg)
{
    int ret = 0;
    uint8_t datmp, regv;
    int coeff;

    ret = es8311_openbus(&es8311_cfg);


    // ret |= es8311_write_reg(0x00,0x1F);
    // ret |= es8311_write_reg(0x45,0x00);
    // k+l

    ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG01, 0x30);
    ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG02, 0x00);
    ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG03, 0x10);
    ret |= es8311_write_reg(ES8311_ADC_REG16, 0x24);
    ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG04, 0x10);
    ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG05, 0x00);
    ret |= es8311_write_reg(ES8311_SYSTEM_REG0B, 0x00);
    ret |= es8311_write_reg(ES8311_SYSTEM_REG0C, 0x00);
    ret |= es8311_write_reg(ES8311_SYSTEM_REG10, 0x1F);
    ret |= es8311_write_reg(ES8311_SYSTEM_REG11, 0x7F);
    ret |= es8311_write_reg(ES8311_RESET_REG00, 0x80);

    /*Read Chip ID*/
    uint8_t id1 = es8311_read_reg(ES8311_CHD1_REGFD);
    uint8_t id2 = es8311_read_reg(ES8311_CHD2_REGFE);
    uint16_t id = id1 << 8 | id2;
    uint8_t version = es8311_read_reg(ES8311_CHVER_REGFF);
    LOG_INF("Chip ID:0x%04x - Version: %d", id, version);
    /*
     * Set Codec into Master or Slave mode
     */
    regv = es8311_read_reg(ES8311_RESET_REG00);
    audio_hal_codec_i2s_iface_t *i2s_cfg = &(codec_cfg->i2s_iface);
    switch (i2s_cfg->mode) {
        case AUDIO_HAL_MODE_MASTER:    /* MASTER MODE */
            //LOG_INF("ES8311 in Master mode");
            regv |= 0x40;
            break;
        case AUDIO_HAL_MODE_SLAVE:    /* SLAVE MODE */
            //LOG_INF("ES8311 in Slave mode");
            regv &= 0xBF;
            break;
        default:
            regv &= 0xBF;
    }
    ret |= es8311_write_reg(ES8311_RESET_REG00, regv);
    ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG01, 0x3F);
/*
     * Select clock source for internal mclk
     */
    switch (get_es8311_mclk_src()) {
        case FROM_MCLK_PIN:
            //LOG_INF("[ES8311]: MCLK source from MCLK Pin\r\n");
            regv = es8311_read_reg(ES8311_CLK_MANAGER_REG01);
            regv &= 0x7F;
            ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG01, regv);
            break;
        case FROM_SCLK_PIN:
        //LOG_INF("[ES8311]: MCLK source FROM SCLK Pin\r\n");
            regv = es8311_read_reg(ES8311_CLK_MANAGER_REG01);
            regv |= 0x80;
            ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG01, regv);
            break;
        default:
            regv = es8311_read_reg(ES8311_CLK_MANAGER_REG01);
            regv &= 0x7F;
            ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG01, regv);
            break;
    }
    int sample_fre = 0;
    int mclk_fre = 0;
    switch (i2s_cfg->samples) {
        case AUDIO_HAL_08K_SAMPLES:
            sample_fre = 8000;
            break;
        case AUDIO_HAL_11K_SAMPLES:
            sample_fre = 11025;
            break;
        case AUDIO_HAL_16K_SAMPLES:
            sample_fre = 16000;
            break;
        case AUDIO_HAL_22K_SAMPLES:
            sample_fre = 22050;
            break;
        case AUDIO_HAL_24K_SAMPLES:
            sample_fre = 24000;
            break;
        case AUDIO_HAL_32K_SAMPLES:
            sample_fre = 32000;
            break;
        case AUDIO_HAL_44K_SAMPLES:
            sample_fre = 44100;
            break;
        case AUDIO_HAL_48K_SAMPLES:
            sample_fre = 48000;
            break;
        default:
            LOG_ERR("Unable to configure sample rate %dHz", sample_fre);
            break;
            
    }
    mclk_fre = sample_fre * MCLK_DIV_FRE;
    coeff = get_coeff(mclk_fre, sample_fre);
    if (coeff < 0) {
        LOG_ERR("Unable to configure sample rate %dHz with %dHz MCLK", sample_fre, mclk_fre);
        return -1;
    }
    /*
     * Set clock parammeters
     */
    if (coeff >= 0) {
        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG02) & 0x07;
        regv |= (coeff_div[coeff].pre_div - 1) << 5;
        datmp = 0;
        switch (coeff_div[coeff].pre_multi) {
            case 1:
                datmp = 0;
                break;
            case 2:
                datmp = 1;
                break;
            case 4:
                datmp = 2;
                break;
            case 8:
                datmp = 3;
                break;
            default:
                break;
        }

        if (get_es8311_mclk_src() == FROM_SCLK_PIN) {
            datmp = 3;     /* DIG_MCLK = LRCK * 256 = BCLK * 8 */
        }
        regv |= (datmp) << 3;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG02, regv);

        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG05) & 0x00;
        regv |= (coeff_div[coeff].adc_div - 1) << 4;
        regv |= (coeff_div[coeff].dac_div - 1) << 0;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG05, regv);

        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG03) & 0x80;
        regv |= coeff_div[coeff].fs_mode << 6;
        regv |= coeff_div[coeff].adc_osr << 0;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG03, regv);

        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG04) & 0x80;
        regv |= coeff_div[coeff].dac_osr << 0;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG04, regv);

        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG07) & 0xC0;
        regv |= coeff_div[coeff].lrck_h << 0;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG07, regv);

        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG08) & 0x00;
        regv |= coeff_div[coeff].lrck_l << 0;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG08, regv);

        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG06) & 0xE0;
        if (coeff_div[coeff].bclk_div < 19) {
            regv |= (coeff_div[coeff].bclk_div - 1) << 0;
        } else {
            regv |= (coeff_div[coeff].bclk_div) << 0;
        }
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG06, regv);
    }

    /*
     * mclk inverted or not
     */
    if (INVERT_MCLK) {
        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG01);
        regv |= 0x40;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG01, regv);
    } else {
        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG01);
        regv &= ~(0x40);
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG01, regv);
    }
    /*
     * sclk inverted or not
     */
    if (INVERT_SCLK) {
        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG06);
        regv |= 0x20;
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG06, regv);
    } else {
        regv = es8311_read_reg(ES8311_CLK_MANAGER_REG06);
        regv &= ~(0x20);
        ret |= es8311_write_reg(ES8311_CLK_MANAGER_REG06, regv);
    }

    ret |= es8311_write_reg(ES8311_SYSTEM_REG13, 0x10);
    ret |= es8311_write_reg(ES8311_ADC_REG1B, 0x0A);
    ret |= es8311_write_reg(ES8311_ADC_REG1C, 0x6A);

    codec_dac_volume_config_t default_vol_cfg = ES8311_DAC_VOL_CFG_DEFAULT();
    dac_vol_handle = audio_codec_volume_init(&default_vol_cfg);
    uint8_t reg_adc_vol = audio_codec_get_dac_reg_value(dac_vol_handle, dac_vol_handle->user_volume);
    ret |=es8311_write_reg(ES8311_DAC_REG32, reg_adc_vol);
    return ret;
}

int es8311_codec_deinit()
{
//TODO: Implement audio codec volume deinit
    audio_codec_volume_deinit(dac_vol_handle);
    es8311_closebus(&es8311_cfg);
    return 0;
}

int es8311_config_fmt(es_i2s_fmt_t fmt)
{
    int ret = 0;
    uint8_t adc_iface = 0, dac_iface = 0;
    dac_iface = es8311_read_reg(ES8311_SDPIN_REG09);
    adc_iface = es8311_read_reg(ES8311_SDPOUT_REG0A);
    switch (fmt) {
        case AUDIO_HAL_I2S_NORMAL:
            dac_iface &= 0xFC;
            adc_iface &= 0xFC;
            break;
        case AUDIO_HAL_I2S_LEFT:
        case AUDIO_HAL_I2S_RIGHT:
            adc_iface &= 0xFC;
            dac_iface &= 0xFC;
            adc_iface |= 0x01;
            dac_iface |= 0x01;
            break;
        case AUDIO_HAL_I2S_DSP:
            adc_iface &= 0xDC;
            dac_iface &= 0xDC;
            adc_iface |= 0x03;
            dac_iface |= 0x03;
            break;
        default:
            dac_iface &= 0xFC;
            adc_iface &= 0xFC;
            break;
    }
    ret |= es8311_write_reg(ES8311_SDPIN_REG09, dac_iface);
    ret |= es8311_write_reg(ES8311_SDPOUT_REG0A, adc_iface);

    return ret;
}

int es8311_set_bits_per_sample(audio_hal_iface_bits_t bits)
{
    int ret = 0;
    uint8_t adc_iface = 0, dac_iface = 0;
    dac_iface = es8311_read_reg(ES8311_SDPIN_REG09);
    adc_iface = es8311_read_reg(ES8311_SDPOUT_REG0A);
    switch (bits) {
        case AUDIO_HAL_BIT_LENGTH_16BITS:
            dac_iface |= 0x0c;
            adc_iface |= 0x0c;
            //LOG_INF("ES8311 length 16bits\r\n");
            break;
        case AUDIO_HAL_BIT_LENGTH_24BITS:
            //LOG_INF("ES8311 length 24bits\r\n");
            break;
        case AUDIO_HAL_BIT_LENGTH_32BITS:
            dac_iface |= 0x10;
            adc_iface |= 0x10;
            //LOG_INF("ES8311 length 32bits\r\n");
            break;
        default:
            dac_iface |= 0x0c;
            adc_iface |= 0x0c;
            break;

    }
    ret |= es8311_write_reg(ES8311_SDPIN_REG09, dac_iface);
    ret |= es8311_write_reg(ES8311_SDPOUT_REG0A, adc_iface);

    return ret;
}

int es8311_codec_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    int ret = 0;
    ret |= es8311_set_bits_per_sample(iface->bits);
    ret |= es8311_config_fmt(iface->fmt);
    return ret;
}

int es8311_codec_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    int ret = 0;
    es_module_t es_mode = ES_MODULE_MIN;
    switch (mode) {
        case AUDIO_HAL_CODEC_MODE_ENCODE:
            es_mode  = ES_MODULE_ADC;
            //LOG_INF("ES8311 mode ADC Encode\r\n");
            break;
        case AUDIO_HAL_CODEC_MODE_LINE_IN:
            es_mode  = ES_MODULE_LINE;
            //LOG_INF("ES8311 mode LINE IN\r\n");
            break;
        case AUDIO_HAL_CODEC_MODE_DECODE:
            es_mode  = ES_MODULE_DAC;
            //LOG_INF("ES8311 mode DAC Decode\r\n");
            break;
        case AUDIO_HAL_CODEC_MODE_BOTH:
            es_mode  = ES_MODULE_ADC_DAC;
            //LOG_INF("ES8311 mode ADC_DAC\r\n");
            break;
        default:
            es_mode = ES_MODULE_DAC;
            //LOG_WRN("Codec mode not support, default is decode mode");
            break;
    }
    if (ctrl_state == AUDIO_HAL_CTRL_START) {
        ret |= es8311_start(es_mode);
        //LOG_INF("The codec start working\r\n");
    } else {
        //LOG_WRN("The codec is about to stop\r\n");
        ret |= es8311_stop(es_mode);
    }
    return ret;
}

int es8311_start(es_module_t mode)
{
    int ret = 0;
    uint8_t adc_iface = 0, dac_iface = 0;

    dac_iface = es8311_read_reg(ES8311_SDPIN_REG09) & 0xBF;
    adc_iface = es8311_read_reg(ES8311_SDPOUT_REG0A) & 0xBF;
    adc_iface |= BIT(6);
    dac_iface |= BIT(6);

    if (mode == ES_MODULE_LINE) {
        //LOG_ERR("The codec es8311 doesn't support ES_MODULE_LINE mode");
        return -1;
    }
    else
    {
       //LOG_INF("[ES8311]: MODE LINE\r\n");
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        adc_iface &= ~(BIT(6));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        dac_iface &= ~(BIT(6));
    }
    ret |= es8311_write_reg(ES8311_SDPIN_REG09, dac_iface);
    ret |= es8311_write_reg(ES8311_SDPOUT_REG0A, adc_iface);

    ret |= es8311_write_reg(ES8311_ADC_REG17, 0xBF);// Chuan
    ret |= es8311_write_reg(ES8311_SYSTEM_REG0E, 0x02); //Chuan
    ret |= es8311_write_reg(ES8311_SYSTEM_REG12, 0x00); // Nghi ngo, Nhung chac la chuan thoi // ban dau la 0x28
    ret |= es8311_write_reg(ES8311_SYSTEM_REG14, 0x10 /*| 0x80*/);
    /*
     * pdm dmic enable or disable
     */
    int regv = 0;
    if (IS_DMIC) {
        regv = es8311_read_reg(ES8311_SYSTEM_REG14);
        regv |= 0x40;
        ret |= es8311_write_reg(ES8311_SYSTEM_REG14, regv);
    } else {
        regv = es8311_read_reg(ES8311_SYSTEM_REG14);
        regv &= ~(0x40);
        ret |= es8311_write_reg(ES8311_SYSTEM_REG14, regv);
    }
    ret |= es8311_write_reg(ES8311_SYSTEM_REG0D, 0x01);
    ret |= es8311_write_reg(ES8311_ADC_REG15, 0x40);
    ret |= es8311_write_reg(ES8311_DAC_REG37, 0x48);
    ret |= es8311_write_reg(ES8311_GP_REG45, 0x00); 

    // ret |= es8311_write_reg(ES8311_ADC_REG17, 0xBF); 
    // ret |= es8311_write_reg(ES8311_DAC_REG32, 0xC1);
    /* set internal reference signal (ADCL + DACR) */

    uint8_t dac_source = 0x00;
#if(CONFIG_AUDIO_DEV == 2)
    dac_source = dac_source | 0x80;
#endif
    ret |= es8311_write_reg(ES8311_GPIO_REG44, dac_source);
    //es8311_read_all();
    return ret;
}


int es8311_stop(es_module_t mode)
{
    int ret = 0;
    es8311_suspend();
    return ret;
}

/**
 * @brief Set voice volume
 *
 * @note Register values. 0x00: -95.5 dB, 0x5B: -50 dB, 0xBF: 0 dB, 0xFF: 32 dB
 * @note Accuracy of gain is 0.5 dB
 *
 * @param volume: voice volume (0~100)
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
int es8311_codec_set_voice_volume(int volume)
{
    int res = 0;
    uint8_t reg = 0;
    /*get current volume value*/
    reg = audio_codec_get_dac_reg_value(dac_vol_handle, volume);
    res = es8311_write_reg(ES8311_DAC_REG32, reg);
    LOG_INF("Set volume:%.2d reg_value:0x%.2x dB:%.1f", dac_vol_handle->user_volume, reg,
             audio_codec_cal_dac_volume(dac_vol_handle));
    //pair_ultilities_flash_save_volume_cfg(dac_vol_handle->user_volume);
    return res;
}

int es8311_codec_get_voice_volume(int *volume)
{
    int res = 0;
    int regv = 0;
    regv = es8311_read_reg(ES8311_DAC_REG32);
    if (regv == -1) {
        *volume = 0;
        res = -1;
    } else {
        if (regv == dac_vol_handle->reg_value) {
            *volume = dac_vol_handle->user_volume;
        } else {
            *volume = 0;
            res = -1;
        }
    }
    LOG_INF("Get volume:%d reg_value:0x%02x", *volume, regv);
    return res;
}

int es8311_set_voice_mute(bool enable)
{
    LOG_DBG("Es8311 mute:%d", enable);
    es8311_mute(enable);
    return 0;
}
int es8311_get_voice_mute(int *mute)
{
    int res = 0;
    uint8_t reg = 0;
    res = es8311_read_reg(ES8311_DAC_REG31);
    if (res != -1) {
        reg = (res & 0x20) >> 5;
    }
    *mute = reg;
    return res;
}

int es8311_set_mic_gain(es8311_mic_gain_t gain_db)
{
    int res = 0;
    res = es8311_write_reg(ES8311_ADC_REG16, gain_db); // MIC gain scale
    LOG_INF("Set MIC gain: %d", gain_db);
    return res;
}
es8311_mic_gain_t es8311_get_mic_gain(void)
{
    int res = 0;
    res = es8311_read_reg(ES8311_ADC_REG16);
    res = res & 0x07; /*Lay 3 bit dau*/
    LOG_INF("Get MIC gain: %d", res);
    return res;
}
void es8311_read_all()
{
    for (int i = 0; i < 0x45; i++) {
        uint8_t reg = es8311_read_reg(i);
        LOG_INF("REG:%02x, %02x\n", reg, i);
    }
}
int8_t get_es8311_mclk_src(void)
{
    return 1;
}
