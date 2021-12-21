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
#include <keep_alive.h>

#define SDRONE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN
#define SDRONE_MAX_PUBLISHERS     2
#define SDRONE_MAX_OBSERVERS      4

typedef esp_err_t (*sdrone_wsserver_receive_frame_fn_t)(httpd_ws_frame_t* ws_pkt);

typedef struct {
	httpd_uri_t handler;
	sdrone_wsserver_receive_frame_fn_t receive_cb;
	int sockfd[SDRONE_MAX_OBSERVERS];
} sdrone_httpd_uri_t;

typedef sdrone_httpd_uri_t* sdrone_httpd_uri_handle_t;

typedef struct {
	httpd_handle_t server;
	wss_keep_alive_t keep_alive;
	sdrone_httpd_uri_handle_t uri_handlers[SDRONE_MAX_PUBLISHERS];
} sdrone_wsserver_state_t;
typedef sdrone_wsserver_state_t* sdrone_wsserver_handle_t;

void sdrone_wsserver_init(sdrone_wsserver_handle_t wsserver_handle);
void sdrone_wsserver_add_uri_handle(sdrone_httpd_uri_handle_t uri);
void sdrone_wsserver_remove_uri_handle(sdrone_httpd_uri_handle_t uri);
void sdrone_wsserver_send_data(sdrone_httpd_uri_handle_t uri_handler, httpd_ws_frame_t* frame);

#endif /* COMPONENTS_DRIVERS_WSSERVER_INCLUDE_WSSERVER_H_ */
