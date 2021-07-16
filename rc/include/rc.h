/*
 * ppm8.h
 *
 *  Created on: 23 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_RC_INCLUDE_RC_H_
#define COMPONENTS_RC_INCLUDE_RC_H_
#include <stdint.h>
#include <esp_err.h>
#include <driver/rmt.h>

#define RC_CHANNEL            0                  /* Channel input (0-7) for receiver */
#define RC_GPIO_NUM           23                 /* GPIO number for receiver */
#define RC_CLK_DIV            8                  /* Clock divider */
#define RC_TICK_US            (80/RC_CLK_DIV)    /* Number of Ticks for us */
#define RC_PPM_TIMEOUT_US     3500               /* min PPM silence (us) */
#define RC_BUFF_BLOCK_NUM     4                  /* Memory Blocks */
#define RC_TICK_TRASH         100                /* Interference */
#define RC_MAX_CHANNELS       8

#define RC_THROTTLE 1
#define RC_ROLL     0
#define RC_PITCH    2
#define RC_YAW      3
#define RC_AUX1     5
#define RC_AUX2     4
#define RC_AUX3     6
#define RC_AUX4     7

typedef enum {
        RC_TXRX_IGNORE = 0,
        RC_TXRX_TRANSMITTED = 1,
        RC_TXRX_RECEIVED = 2
} rc_txrx_signal_t;

typedef struct {
	uint16_t min;
	uint16_t max;
	uint16_t center;
	uint16_t value;
} rc_stick_range_t;

typedef struct {
	uint16_t raw[RC_MAX_CHANNELS];
	int16_t norm[RC_MAX_CHANNELS];
	volatile rc_txrx_signal_t txrx_signal;
} rc_data_t;
typedef struct {
	rmt_config_t config;
	rc_stick_range_t rc_channels_range[RC_MAX_CHANNELS];
	rc_data_t data;
} rc_t;
typedef rc_t* rc_handle_t;

esp_err_t rc_init(rc_handle_t rc_handle);
esp_err_t rc_start(rc_handle_t rc_handle);
esp_err_t rc_stop(rc_handle_t rc_handle);

#endif /* COMPONENTS_RC_INCLUDE_RC_H_ */
