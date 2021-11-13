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

typedef enum {
	RC_THROTTLE = 0,
	RC_ROLL,
	RC_PITCH,
	RC_YAW,
	RC_VRA,
	RC_VRB,
	RC_SWA,
	RC_SWB,
	RC_SWC,
	RC_SWD
} RC_STICK;

typedef enum {
        RC_TXRX_IGNORE = 0,
        RC_TXRX_TRANSMITTED = 1,
        RC_TXRX_RECEIVED = 2,
        RC_TXRX_DATA_NOT_RECEIVED = 3
} rc_txrx_signal_t;

typedef enum {
        RC_NOT_CONNECTED = 0,
        RC_CONNECTED = 1,
} rc_state_t;

typedef struct {
	uint16_t min;
	uint16_t max;
	uint16_t center;
	uint16_t value;
} rc_stick_range_t;

#define RC_MAX_CHANNELS       10

typedef struct {
	uint16_t raw[RC_MAX_CHANNELS];
	int16_t norm[RC_MAX_CHANNELS];
	volatile rc_txrx_signal_t txrx_signal;
} rc_data_t;

typedef struct {
	void* dev_config;
    esp_err_t (*init)(void* rc_handle);
    esp_err_t (*start)(void* rc_handle);
    esp_err_t (*stop)(void* rc_handle);
	rc_stick_range_t rc_channels_range[RC_MAX_CHANNELS];
	rc_data_t data;
	volatile rc_state_t state;
} rc_t;
typedef rc_t* rc_handle_t;

esp_err_t rc_init(rc_handle_t rc_handle);
esp_err_t rc_start(rc_handle_t rc_handle);
esp_err_t rc_stop(rc_handle_t rc_handle);

#endif /* COMPONENTS_RC_INCLUDE_RC_H_ */
