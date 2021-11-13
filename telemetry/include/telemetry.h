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

#define MESSAGE_RC      0x01
#define MESSAGE_RPY     0x02
#define MESSAGE_ACC     0x03
#define MESSAGE_W       0x04
#define MESSAGE_AXIS    0x05
#define MESSAGE_GRAVITY 0x06
#define MESSAGE_V       0x07
#define NUM_MESSAGES    0x07

typedef struct {
    httpd_handle_t hd;
    int fd;
} sdrone_telemetry_gs_socket_t;

typedef struct {
	uint16_t m_type;
	uint32_t m_timestamp;
	union {
		struct {
			int16_t throttle;
			int16_t roll;
			int16_t pitch;
			int16_t yaw;
			int16_t aux1;
			int16_t aux2;
		} rc;
		struct {
			float roll;
			float pitch;
			float yaw;
		} rpy;
		struct {
			float x;
			float y;
			float z;
		} acc;
		struct {
			float x;
			float y;
			float z;
			float thrust;
		} w;
		struct {
			float x;
			float y;
			float z;
			float thrust;
		} axis;
		struct {
			float x;
			float y;
			float z;
		} gravity;
		float vertical_v;
	};
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
