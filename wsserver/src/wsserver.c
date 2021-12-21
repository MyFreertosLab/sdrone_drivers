#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <esp_http_server.h>
#include <wsserver.h>
#include <keep_alive.h>

#define SDRONE_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define SDRONE_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define SDRONE_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL

static const char *TAG = "sdrone_wsserver";
static sdrone_wsserver_handle_t internal_wsserver_handle = NULL;


struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

/***********************************************************************
 ************************ W E B  S O C K E T ***************************
 ***********************************************************************/

static void send_ping(void *arg)
{
    struct async_resp_arg* resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = NULL;
    ws_pkt.len = 0;
    ws_pkt.type = HTTPD_WS_TYPE_PING;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

static void send_pong(void *arg)
{
    struct async_resp_arg* resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = NULL;
    ws_pkt.len = 0;
    ws_pkt.type = HTTPD_WS_TYPE_PONG;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

static esp_err_t sdrone_wsserver_add_client(sdrone_httpd_uri_handle_t uri_handle, int sockfd) {
	for(uint8_t j = 0; j < SDRONE_MAX_OBSERVERS; j++) {
		if(uri_handle->sockfd[j] == sockfd) {
			return ESP_OK;
		}
	}
	for(uint8_t j = 0; j < SDRONE_MAX_OBSERVERS; j++) {
		if(uri_handle->sockfd[j] == -1) {
			uri_handle->sockfd[j] = sockfd;
	        ESP_LOGI(TAG, "ws_handler added a new socket [%d]", sockfd);
	        return ESP_OK;
		}
	}
	return ESP_FAIL;
}
static esp_err_t sdrone_ws_handler(httpd_req_t *req)
{
    sdrone_wsserver_handle_t wsserver_handle = internal_wsserver_handle;
    int sockfd = httpd_req_to_sockfd(req);
    sdrone_httpd_uri_handle_t uri_handle = NULL;

    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        // add new socket for publisher
        for(uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
        	if((wsserver_handle->uri_handlers[i] != NULL) && (strcmp(wsserver_handle->uri_handlers[i]->handler.uri, req->uri) == 0) && wsserver_handle->uri_handlers[i]->handler.method == HTTP_GET) {
        		uri_handle = wsserver_handle->uri_handlers[i];
        		return sdrone_wsserver_add_client(uri_handle, sockfd);
        	}
        }
        ESP_LOGE(TAG, "ws_handler uri_handler not found for uri [%s]", req->uri);
        return ESP_FAIL;
    } else {
    	// search publisher that publish for this socket
        for(uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
        	for(uint8_t j = 0; j < SDRONE_MAX_OBSERVERS; j++) {
        		if((wsserver_handle->uri_handlers[i] != NULL) && (wsserver_handle->uri_handlers[i]->sockfd[j] == sockfd)) {
        			uri_handle = wsserver_handle->uri_handlers[i];
        		}
        	}
        }
        if(uri_handle == NULL) {
            ESP_LOGE(TAG, "ws_handler uri_handler not found for uri [%s] with method [%d]", req->uri, req->method);
        	return ESP_FAIL;
        }
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    // First receive the full ws message
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with [%d]. type=[%d]", ret, ws_pkt.type);
    }
    ESP_LOGI(TAG, "frame len is %d and type [%d]", ws_pkt.len, ws_pkt.type);
    if (ws_pkt.len > 0) {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
    }
    // If it was a PONG, update the keep-alive
    if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
        ESP_LOGI(TAG, "Received PONG message");
        free(buf);
        return wss_keep_alive_client_is_active(wsserver_handle->keep_alive, sockfd);
    } else if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
            ESP_LOGI(TAG, "Received PING message");
            free(buf);
            struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
            resp_arg->hd = wsserver_handle->server;
            resp_arg->fd = sockfd;
            if (httpd_queue_work(wsserver_handle->server, send_pong, resp_arg) == ESP_OK) {
                ESP_LOGI(TAG, "Sent PONG message");
            } else {
                ESP_LOGW(TAG, "Failed to send PONG message");
            }


    } else {
        ESP_LOGI(TAG, "Received packet with message: %s", ws_pkt.payload);
        esp_err_t ret = ESP_OK;
        if(uri_handle->receive_cb != NULL) {
            ret = uri_handle->receive_cb(&ws_pkt);
        }
        free(buf);
        return ret;
    }
    free(buf);
    return ESP_OK;
}

/***********************************************************************
 ************************ W E B  S E R V E R ***************************
 ***********************************************************************/
static esp_err_t wss_open_fd(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "New client connected %d", sockfd);
    sdrone_wsserver_handle_t wsserver_handle = httpd_get_global_user_ctx(hd);
    return wss_keep_alive_add_client(wsserver_handle->keep_alive, sockfd);
}
static void wss_close_fd(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "Client disconnected %d", sockfd);
    sdrone_wsserver_handle_t wsserver_handle = httpd_get_global_user_ctx(hd);
	for (uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
		if(internal_wsserver_handle->uri_handlers[i] != NULL) {
			for (uint8_t j = 0; j < SDRONE_MAX_OBSERVERS; j++) {
				if (internal_wsserver_handle->uri_handlers[i]->sockfd[j] == sockfd) {
					internal_wsserver_handle->uri_handlers[i]->sockfd[j] = -1;
				}
			}
		}
	}
    wss_keep_alive_remove_client(wsserver_handle->keep_alive, sockfd);
}

static void user_global_ctx_free(void* ctx) {
    ESP_LOGI(TAG, "Free user global context");
    sdrone_wsserver_handle_t wsserver_handle = (sdrone_wsserver_handle_t)ctx;
	wsserver_handle->server = NULL;
}
static bool client_not_alive_cb(wss_keep_alive_t h, int fd)
{
    ESP_LOGE(TAG, "Client not alive, closing fd %d", fd);
    sdrone_wsserver_handle_t wsserver_handle = (sdrone_wsserver_handle_t)wss_keep_alive_get_user_ctx(h);
    httpd_sess_trigger_close(wsserver_handle->server, fd);
    return true;
}

static bool check_client_alive_cb(wss_keep_alive_t h, int fd)
{
    ESP_LOGI(TAG, "Checking if client (fd=%d) is alive", fd);
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    sdrone_wsserver_handle_t wsserver_handle = (sdrone_wsserver_handle_t)wss_keep_alive_get_user_ctx(h);
    resp_arg->hd = wsserver_handle->server;
    resp_arg->fd = fd;

    if (httpd_queue_work(resp_arg->hd, send_ping, resp_arg) == ESP_OK) {
        return true;
    }
    ESP_LOGI(TAG, "Checking return false");
    return false;
}

static esp_err_t start_webserver(sdrone_wsserver_handle_t wsserver_handle)
{
    // Prepare keep-alive engine
    wss_keep_alive_config_t keep_alive_config = KEEP_ALIVE_CONFIG_DEFAULT();
    keep_alive_config.max_clients = SDRONE_MAX_OBSERVERS;
    keep_alive_config.client_not_alive_cb = client_not_alive_cb;
    keep_alive_config.check_client_alive_cb = check_client_alive_cb;
    keep_alive_config.user_ctx = (void*)wsserver_handle;
    keep_alive_config.keep_alive_period_ms = 30000;
    keep_alive_config.not_alive_after_ms = 50000;
    wss_keep_alive_t keep_alive = wss_keep_alive_start(&keep_alive_config);
    wsserver_handle->keep_alive = keep_alive;

    // Prepare Httpd server
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.open_fn = wss_open_fd;
    config.close_fn = wss_close_fd;
    config.global_user_ctx = (void*)wsserver_handle;
    config.global_user_ctx_free_fn = user_global_ctx_free;
    config.core_id = 1;
    config.send_wait_timeout = 20;
    config.recv_wait_timeout = 20;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
    	wsserver_handle->server = server;
    	// register uri handlers
    	for(uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
    		if(wsserver_handle->uri_handlers[i] != NULL) {
    			httpd_register_uri_handler(wsserver_handle->server, &wsserver_handle->uri_handlers[i]->handler);
    			ESP_LOGI(TAG, "uri handler [%s] registered", wsserver_handle->uri_handlers[i]->handler.uri);
    		}
    	}
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Error starting server!");
    return ESP_FAIL;
}

static void stop_webserver(sdrone_wsserver_handle_t wsserver_handle) {
	if (wsserver_handle->server != NULL) {
	    wss_keep_alive_stop(wsserver_handle->keep_alive);
		httpd_stop(wsserver_handle->server);
	}
}

/***********************************************************************
 ****************************** W I F I ********************************
 ***********************************************************************/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
	sdrone_wsserver_handle_t wsserver_handle = (sdrone_wsserver_handle_t)arg;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
        start_webserver(wsserver_handle);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
        stop_webserver(wsserver_handle);
    }
}

static void wifi_init_softap(sdrone_wsserver_handle_t wsserver_handle)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
														(void*)wsserver_handle,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SDRONE_WIFI_SSID,
            .ssid_len = strlen(SDRONE_WIFI_SSID),
            .channel = SDRONE_WIFI_CHANNEL,
            .password = SDRONE_WIFI_PASS,
            .max_connection = SDRONE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(SDRONE_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             SDRONE_WIFI_SSID, SDRONE_WIFI_PASS, SDRONE_WIFI_CHANNEL);
}

/***********************************************************************
 ********************* P U B L I C  M E T H O D S **********************
 ***********************************************************************/

void sdrone_wsserver_init(sdrone_wsserver_handle_t wsserver_handle) {

	//
	internal_wsserver_handle = wsserver_handle;

	// init wsserver state
	wsserver_handle->server = NULL;
	for(uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
		wsserver_handle->uri_handlers[i] = NULL;
	}

	// init wifi AP
    wifi_init_softap(wsserver_handle);
}

void sdrone_wsserver_add_uri_handle(sdrone_httpd_uri_handle_t uri) {
	uint8_t pos = 255;
	for(uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
		if(internal_wsserver_handle->uri_handlers[i] == NULL && pos == 255) {
			pos = i;
		} else if(internal_wsserver_handle->uri_handlers[i] == uri) {
			ESP_LOGE(TAG, "Uri handler already exists for uri [%s]! Not added", uri->handler.uri);
			return;
		}
	}
	internal_wsserver_handle->uri_handlers[pos] = uri;

	// init sockets for this handler
	for(uint8_t i = 0; i < SDRONE_MAX_OBSERVERS; i++) {
		uri->sockfd[i] = -1;
	}

	// set default handler callback if it's NULL
	if(uri->handler.handler == NULL) {
		uri->handler.handler  = sdrone_ws_handler;
	}

	ESP_LOGI(TAG, "uri handler [%s] added at position %d", uri->handler.uri, pos);
	if(internal_wsserver_handle->server != NULL) {
		httpd_register_uri_handler(internal_wsserver_handle->server, &uri->handler);
		ESP_LOGI(TAG, "uri handler [%s] registered", uri->handler.uri);
	}
}

void sdrone_wsserver_remove_uri_handle(sdrone_httpd_uri_handle_t uri) {
	httpd_unregister_uri_handler(internal_wsserver_handle->server, uri->handler.uri, uri->handler.method);
	for(uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
		if(internal_wsserver_handle->uri_handlers[i] == uri) {
			internal_wsserver_handle->uri_handlers[i] = NULL;
		}
	}
}

void sdrone_wsserver_send_data(sdrone_httpd_uri_handle_t uri_handler, httpd_ws_frame_t* frame) {
	for(uint8_t i = 0; i < SDRONE_MAX_PUBLISHERS; i++) {
		if(internal_wsserver_handle->uri_handlers[i] == uri_handler) {
			for(uint8_t j = 0; j < SDRONE_MAX_OBSERVERS; j++) {
				if(internal_wsserver_handle->uri_handlers[i]->sockfd[j] >= 0) {
					httpd_ws_send_frame_async(internal_wsserver_handle->server, internal_wsserver_handle->uri_handlers[i]->sockfd[j], frame);
				}
			}
		}
	}
}

