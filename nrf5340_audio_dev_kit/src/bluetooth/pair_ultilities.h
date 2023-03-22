#ifndef _PAIR_UTILITIES_H_
#define _PAIR_UTILITIES_H_
#include <stdint.h>
#include <stdbool.h>
#include "le_audio.h"

#define MIXER_DEFAULT_TOKEN 0x6996abcd
#define MIXER_DEFAULT_RESPONSE_TOKEN    0x12345678

#define DEVICE_TYPE_GATEWAY_MIXER   0xAA
#define DEVICE_TYPE_SPEAKER         0xBB

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

typedef int(*ble_custom_nus_fair_done_callback)(le_audio_receive_cb rec_cb);

bool pair_utilities_get_mac_from_name(char *p_name, uint8_t *p_mac_address);

int ultilities_load_mac();

uint8_t *ultilities_get_mac();

uint8_t pair_ultilities_flash_save_gateway_info(uint8_t *gateway_mac);

uint8_t pair_ultilities_flash_read_gateway_info(uint8_t *gateway_mac);

uint8_t pair_ultilities_flash_init();
#endif