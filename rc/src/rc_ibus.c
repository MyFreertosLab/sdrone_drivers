/*
 * rc_ibus.c
 *
 *  Created on: 8 nov 2021
 *      Author: andrea
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <driver/gpio.h>
#include <driver/uart.h>
#include <rc_ibus.h>
#include <esp_log.h>

#define RC_IBUS_GPIO_NUM           23
#define RC_IBUS_UART_NUM           UART_NUM_1
#define RC_IBUS_BUF_SIZE           (68)

#define RC_IBUS_DEVICE_CONFIG(rc_handle) ((rc_ibus_config_t*)rc_handle->dev_config)

typedef struct {
    uint8_t uart_num;
    uint8_t gpio_num;
    uint8_t buffer[RC_IBUS_BUF_SIZE];
    uart_config_t uart_config;
    QueueHandle_t uart_queue;
} rc_ibus_config_t;

static rc_ibus_config_t rc_ibus_config = {
		.uart_num = RC_IBUS_GPIO_NUM,
		.gpio_num = RC_IBUS_GPIO_NUM,
		.uart_config = {
				  .baud_rate = 115200,
				  .data_bits = UART_DATA_8_BITS,
				  .parity = UART_PARITY_DISABLE,
				  .stop_bits = UART_STOP_BITS_1,
				  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
				  .source_clk = UART_SCLK_APB,
				 }
};

static const char *TAG = "rc_ibus";

typedef enum {
	C2S_ROLL = 0,
	C2S_PITCH,
	C2S_THROTTLE,
	C2S_YAW,
	C2S_VRA,
	C2S_VRB,
	C2S_SWA,
	C2S_SWB,
	C2S_SWC,
	C2S_SWD
} CHANNEL_TO_STICK;

static void rc_ibus_read_data(rc_handle_t rc_handle) {
	uint16_t chan_val[15] = { 0 };

	// read data
	uart_flush(RC_IBUS_UART_NUM);
	const int rxBytes = uart_read_bytes(RC_IBUS_UART_NUM,
	RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer, RC_IBUS_BUF_SIZE,
			pdMS_TO_TICKS(2000));

	// data received
	if (rxBytes > 0) {
		uint16_t streamStartAt = RC_IBUS_BUF_SIZE + 1;

		// detect start of stream
		for (uint16_t i = 0; i < RC_IBUS_BUF_SIZE - 1; i++) {
			if (RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[i] == 0x20
					&& RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[i + 1]
							== 0x40) {
				streamStartAt = i;
				break;
			}
		}

		// load data
		uint16_t checksum = 0;
		memset(chan_val, 0, sizeof(chan_val));
		checksum = 0;
		if (streamStartAt < (RC_IBUS_BUF_SIZE - 34)) {
			for (uint8_t i = 0; i < 14; i++) {
				chan_val[i] =
						(uint16_t) (RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[streamStartAt
								+ 2 * i + 3] << 8)
								| (uint16_t) (RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[streamStartAt
										+ 2 * (i + 1)]);
				checksum +=
						((RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[streamStartAt
								+ 2 * i + 3])
								+ (RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[streamStartAt
										+ 2 * (i + 1)]));
			}
			chan_val[14] =
					((RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[streamStartAt
							+ 31] << 8)
							| (uint16_t) (RC_IBUS_DEVICE_CONFIG(rc_handle)->buffer[streamStartAt
									+ 30]));
			checksum += (0x20 + 0x40);
			checksum = 0xFFFF - checksum;

			// check checksum and assign data
			if (checksum == chan_val[14]) {
				// failsafe on rc chans 0-4 at -98%
				if (chan_val[0] < 1020 && chan_val[1] < 1020
						&& chan_val[2] < 1020 && chan_val[3] < 1020) {
					rc_handle->data.txrx_signal = RC_TXRX_DATA_NOT_RECEIVED;
				} else { // data ok
					rc_handle->data.raw[RC_THROTTLE] = chan_val[C2S_THROTTLE];
					rc_handle->data.raw[RC_ROLL] = chan_val[C2S_ROLL];
					rc_handle->data.raw[RC_PITCH] = chan_val[C2S_PITCH];
					rc_handle->data.raw[RC_YAW] = chan_val[C2S_YAW];
					rc_handle->data.raw[RC_VRA] = chan_val[C2S_VRA];
					rc_handle->data.raw[RC_VRB] = chan_val[C2S_VRB];
					rc_handle->data.raw[RC_SWA] = chan_val[C2S_SWA];
					rc_handle->data.raw[RC_SWB] = chan_val[C2S_SWB];
					rc_handle->data.raw[RC_SWC] = chan_val[C2S_SWC];
					rc_handle->data.raw[RC_SWD] = chan_val[C2S_SWD];
					rc_handle->data.txrx_signal = RC_TXRX_TRANSMITTED;
				}
			} else {
				rc_handle->data.txrx_signal = RC_TXRX_DATA_NOT_RECEIVED;
			}
		}
	} else {
		rc_handle->data.txrx_signal = RC_TXRX_DATA_NOT_RECEIVED;
	}
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t rc_ibus_init(rc_handle_t rc_handle) {
	rc_handle->dev_config = (void*)&rc_ibus_config;
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Install UART driver, and get the queue.
    uart_driver_install(RC_IBUS_UART_NUM, 136, 0, 0, NULL, 0);
    uart_param_config(RC_IBUS_UART_NUM, &(RC_IBUS_DEVICE_CONFIG(rc_handle)->uart_config));

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(RC_IBUS_UART_NUM, UART_PIN_NO_CHANGE, RC_IBUS_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	return ESP_OK;
}

esp_err_t rc_ibus_start(rc_handle_t rc_handle) {
	rc_ibus_read_data(rc_handle);
	return ESP_OK;
}

esp_err_t rc_ibus_stop(rc_handle_t rc_handle) {
	return ESP_OK;
}


