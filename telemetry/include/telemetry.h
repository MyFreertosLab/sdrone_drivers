/*
 * telemetry.h
 *
 *  Created on: 17 ott 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_DRIVERS_TELEMETRY_INCLUDE_TELEMETRY_H_
#define COMPONENTS_DRIVERS_TELEMETRY_INCLUDE_TELEMETRY_H_
#include <stdint.h>
#include <esp_err.h>
#include "esp_wifi.h"
#include <esp_http_server.h>

#define SDRONE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

typedef struct {
    httpd_handle_t hd;
    int fd;
} sdrone_telemetry_gs_socket_t;

typedef struct {
	int16_t rc_throttle;
	int16_t rc_roll;
	int16_t rc_pitch;
	int16_t rc_yaw;
	int16_t rc_aux1;
	int16_t rc_aux2;
	float roll;
	float pitch;
	float yaw;
	float acc_x;
	float acc_y;
	float acc_z;
	float w_x;
	float w_y;
	float w_z;
	float w_thrust;
	float ax_x;
	float ax_y;
	float ax_z;
	float ax_thrust;
} sdrone_telemetry_data_t;

typedef struct {
	httpd_handle_t server;
	sdrone_telemetry_gs_socket_t sockets[SDRONE_MAX_STA_CONN];
	sdrone_telemetry_data_t data;
	uint8_t cursor;
} sdrone_telemetry_state_t;
typedef sdrone_telemetry_state_t* sdrone_telemetry_handle_t;

void sdrone_telemetry_init(sdrone_telemetry_handle_t telemetry_handle);
void sdrone_telemetry_send_data(sdrone_telemetry_handle_t telemetry_handle);

#endif /* COMPONENTS_DRIVERS_TELEMETRY_INCLUDE_TELEMETRY_H_ */
