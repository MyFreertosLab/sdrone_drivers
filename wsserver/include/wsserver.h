/*
 * wsserver.h
 *
 *  Created on: 17 ott 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_DRIVERS_WSSERVER_INCLUDE_WSSERVER_H_
#define COMPONENTS_DRIVERS_WSSERVER_INCLUDE_WSSERVER_H_
#include <stdint.h>
#include <esp_err.h>
#include "esp_wifi.h"
#include <esp_http_server.h>

#define SDRONE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

typedef struct {
    httpd_handle_t hd;
    int fd;
} sdrone_wsserver_gs_socket_t;

typedef struct {
	httpd_handle_t server;
	sdrone_wsserver_gs_socket_t sockets[SDRONE_MAX_STA_CONN];
	uint8_t cursor;
} sdrone_wsserver_state_t;
typedef sdrone_wsserver_state_t* sdrone_wsserver_handle_t;

void sdrone_wsserver_init(sdrone_wsserver_handle_t wsserver_handle);
void sdrone_wsserver_send_data(sdrone_wsserver_handle_t wsserver_handle);

#endif /* COMPONENTS_DRIVERS_WSSERVER_INCLUDE_WSSERVER_H_ */
