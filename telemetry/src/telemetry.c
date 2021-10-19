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
#include <telemetry.h>

#define SDRONE_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define SDRONE_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define SDRONE_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL

static const char *TAG = "sdrone_telemetry";
static sdrone_telemetry_handle_t local_telemetry_handle = NULL;

/***********************************************************************
 ************************ W E B  S O C K E T ***************************
 ***********************************************************************/
static void ws_async_send(void *arg)
{
    sdrone_telemetry_handle_t telemetry_handle = arg;
	if(telemetry_handle == NULL) {
    	ESP_LOGE(TAG, "TELEMETRY_HANDLE NULL???");
		return;
	}

    httpd_handle_t hd = telemetry_handle->sockets[telemetry_handle->cursor].hd;
    int fd = telemetry_handle->sockets[telemetry_handle->cursor].fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)&telemetry_handle->data;
    ws_pkt.len = sizeof(telemetry_handle->data);
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    ws_pkt.final = true;

    // send data
	esp_err_t ret = httpd_ws_send_frame_async(hd, fd, &ws_pkt);

	// deregister socket (how to close it?)
	if(ret != ESP_OK) {
		telemetry_handle->sockets[telemetry_handle->cursor].hd = NULL;
		telemetry_handle->sockets[telemetry_handle->cursor].fd = 0;
	}
}

/***********************************************************************
 ************************ W E B  S E R V E R ***************************
 ***********************************************************************/
static esp_err_t ws_get_handler(httpd_req_t *req)
{
	// register socket
	sdrone_telemetry_handle_t telemetry_handle = local_telemetry_handle;
	if(telemetry_handle == NULL) {
    	ESP_LOGE(TAG, "TELEMETRY_HANDLE NULL???");
		return ESP_FAIL;
	}
	for(uint8_t i = 0; i < SDRONE_MAX_STA_CONN; i++) {
		if(telemetry_handle->sockets[i].hd == NULL) {
			telemetry_handle->sockets[i].hd = req->handle;
			telemetry_handle->sockets[i].fd = httpd_req_to_sockfd(req);
			break;
		}
	}

	// receive packet
    httpd_ws_frame_t ws_pkt;
	memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
	ws_pkt.type = HTTPD_WS_TYPE_TEXT;

	// determine the actual packet length
    ws_pkt.payload = malloc(255); // extra byte for string NULL-termination
    assert(ws_pkt.payload);

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 255);
    if (ret != ESP_OK) {
    	ESP_LOGE(TAG, "httpd_ws_recv_frame failed1: %s", esp_err_to_name(ret));
        free(ws_pkt.payload);
        return ret;
    }

    ESP_LOGI(TAG, "Websocket input is, type=%d, len=%d:", ws_pkt.type, ws_pkt.len);

    free(ws_pkt.payload);

    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    return ESP_OK;
}


static esp_err_t wss_open_fd(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "New client connected %d", sockfd);
    return ESP_OK;
}
static void wss_close_fd(httpd_handle_t hd, int sockfd)
{
    ESP_LOGI(TAG, "Client disconnected %d", sockfd);
}

static httpd_handle_t start_webserver(sdrone_telemetry_handle_t telemetry_handle)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.open_fn = wss_open_fd;
    config.close_fn = wss_close_fd;

    httpd_uri_t ws_get = {
            .uri        = "/ws",
            .method     = HTTP_GET,
            .handler    = ws_get_handler,
            .user_ctx   = (void*)telemetry_handle,
            .is_websocket = true
    };

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ws_get);
        return server;
    }
    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server) {
	if (server != NULL) {
		httpd_stop(server);
	}
}

/***********************************************************************
 ****************************** W I F I ********************************
 ***********************************************************************/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
	sdrone_telemetry_handle_t telemetry_handle = (sdrone_telemetry_handle_t)arg;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
        telemetry_handle->server = start_webserver(telemetry_handle);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
        stop_webserver(telemetry_handle->server);
        telemetry_handle->server = NULL;
    }
}

static void wifi_init_softap(sdrone_telemetry_handle_t telemetry_handle)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
														(void*)telemetry_handle,
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
void sdrone_telemetry_init(sdrone_telemetry_handle_t telemetry_handle) {
	local_telemetry_handle = telemetry_handle;
	// init telemetry state
	telemetry_handle->server = NULL;
	for(uint8_t i = 0; i < SDRONE_MAX_STA_CONN; i++) {
		telemetry_handle->sockets[i].fd = 0;
		telemetry_handle->sockets[i].hd = NULL;
	}

	// init wifi AP
    wifi_init_softap(telemetry_handle);
    ESP_LOGI(TAG, "Telemetry Data Size: [%d]", sizeof(telemetry_handle->data));
}
void sdrone_telemetry_send_data(sdrone_telemetry_handle_t telemetry_handle) {
	for(uint8_t i = 0; i < SDRONE_MAX_STA_CONN; i++) {
		if(telemetry_handle->sockets[i].hd != NULL) {
			telemetry_handle->cursor = i;
			httpd_queue_work(telemetry_handle->sockets[i].hd, ws_async_send, telemetry_handle);
		}
	}
}

