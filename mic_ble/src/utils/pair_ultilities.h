#ifndef _PAIR_UTILITIES_H_
#define _PAIR_UTILITIES_H_
#include <stdint.h>
#include <stdbool.h>
#include "le_audio.h"
//#include "es8311.h"
#include "hw_codec.h"
#define MIXER_DEFAULT_TOKEN 0x6996abcd
#define MIXER_DEFAULT_RESPONSE_TOKEN    0x12345678



/**/
#define MIXER_MESSAGE_ID_PAIR   0x02
#define MIXER_MESSAGE_ID_RESPONSE_PAIR  0x03

typedef struct ble_custom_nus
{
    /* data */
    uint8_t msg_id;
    uint8_t device_type;
    uint8_t mac[6];
    uint32_t token;
}pair_info_request_key_t;

typedef struct {
    uint8_t gateway_mac[6];
    uint8_t device_type;
    uint8_t message_id;
    uint32_t reponse_msg_token;
}pair_response_message_t;

typedef union
{
    struct{
        uint8_t device_mac[6];
        uint8_t device_type;
        uint8_t request_to_speak;
        uint8_t priority_level;
    }gw_refined;
    uint8_t gw_raw[9];
}gateway_data_t;

typedef union
{
    struct pair_ultilities
    {
        /* data */
        gateway_data_t gateway_data[10];
        uint8_t total_device;
        uint8_t selected_device;
    }refined;
    uint8_t raw[92];
}gateway_flash_data_t;

typedef int(*ble_custom_nus_fair_done_callback)(le_audio_receive_cb rec_cb,le_audio_timestamp_cb ts_cb);

bool pair_utilities_get_mac_from_name(char *p_name, uint8_t *p_mac_address);

int ultilities_load_mac();

uint8_t *ultilities_get_mac();

uint8_t pair_ultilities_flash_save_gateway_info(uint8_t *gateway_mac);

uint8_t pair_ultilities_flash_read_gateway_info(uint8_t *gateway_mac);

uint8_t pair_ultilities_flash_init();

uint8_t pair_ultilities_flash_save_volume_cfg(uint8_t *volume_config);

uint8_t pair_ultilities_flash_read_volume_cfg(uint8_t *volume_config);

uint8_t* pair_ultitities_get_gateway_info();

void pair_ultitlities_set_device_name(char* P_pre_name);

uint32_t GetHexNumberFromString(uint16_t BeginAddress, char* Buffer , uint32_t len);

uint8_t ultilities_flash_get_reset(void);

uint8_t ultilities_flash_set_reset_val(bool shutdown);


uint8_t pair_ultilities_gateway_pair_read(gateway_flash_data_t *gateway_mac);

int pair_ultilities_gateway_pair_write(uint8_t *new_gateway, uint8_t device_type);

gateway_flash_data_t* pair_ultilities_gateway_pair_load(void);

uint8_t pair_ultilities_gateway_pair_get(void);

uint8_t pair_ultilities_change_pair_gateway(uint8_t *target_mac);
#endif