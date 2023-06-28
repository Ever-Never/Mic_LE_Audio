/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _STREAMCTRL_H_
#define _STREAMCTRL_H_

#include <stddef.h>
#include <zephyr/kernel.h>


typedef enum
{
	AUDIO_STATE_EVT_INIT,
	AUDIO_STATE_EVT_START_ADV,
	AUDIO_STATE_EVT_FOUND_VALID_SRC,
	AUDIO_STATE_EVT_RECEIVE_SRC_DATA,
	AUDIO_STATE_EVT_LOSS_SYNC,
	AUDIO_STATE_EVT_ENTER_PAIR,
	AUDIO_STATE_EVT_EXIT_PAIR
}audio_state_event_t;
typedef enum
{
	AUDIO_STATE_INIT,
	AUDIO_STATE_FIND_VALID_DEVICE,
	AUDIO_STATE_SYNCING_SRC,
	AUDIO_STATE_SYNC_AND_RECEIVE_SRC_DATA,
	AUDIO_STATE_PAIR_STATE
}audio_state_t;
typedef struct 
{
	audio_state_event_t evt;
	uint8_t data[60];
	uint8_t data_len;
}audio_state_data_t;



/* State machine states for peer/stream */
enum stream_state {
	STATE_STREAMING,
	STATE_PAUSED,
};

/** @brief Get current streaming state
 *
 * @return      strm_state enum value
 */
uint8_t stream_state_get(void);

/** @brief Send encoded data over the stream
 *
 * @param data		Data to send
 * @param size		Size of data
 * @param num_ch	Number of audio channels
 */
void streamctrl_encoded_data_send(void const *const data, size_t size, uint8_t num_ch);

/** @brief Init internal functionality and start streamctrl
 *
 *  @return 0 if successful.
 */
int streamctrl_start(void);
/** @brief 
 *
 *  @return 
 */
int audio_state_publish_event(audio_state_data_t audio_state);


#endif /* _STREAMCTRL_H_ */
