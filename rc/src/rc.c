/*
 * ppm8.c
 *
 *  Created on: 23 feb 2021
 *      Author: andrea
 */
#include <string.h>
#include <rc.h>
#include <driver/gpio.h>
#include "driver/rmt.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>

static 	RingbufHandle_t rb = NULL;

static esp_err_t rc_load_calibration_data(rc_handle_t rc_handle) {
    nvs_handle_t my_handle;
    uint8_t flashed = 0;
    ESP_ERROR_CHECK(nvs_open("RC_CAL", NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_get_u8(my_handle, "FLASHED", &flashed));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_1", &rc_handle->rc_channels_range[0].min));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_2", &rc_handle->rc_channels_range[1].min));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_3", &rc_handle->rc_channels_range[2].min));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_4", &rc_handle->rc_channels_range[3].min));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_5", &rc_handle->rc_channels_range[4].min));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_6", &rc_handle->rc_channels_range[5].min));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_7", &rc_handle->rc_channels_range[6].min));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MIN_8", &rc_handle->rc_channels_range[7].min));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_1", &rc_handle->rc_channels_range[0].max));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_2", &rc_handle->rc_channels_range[1].max));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_3", &rc_handle->rc_channels_range[2].max));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_4", &rc_handle->rc_channels_range[3].max));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_5", &rc_handle->rc_channels_range[4].max));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_6", &rc_handle->rc_channels_range[5].max));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_7", &rc_handle->rc_channels_range[6].max));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_MAX_8", &rc_handle->rc_channels_range[7].max));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_1", &rc_handle->rc_channels_range[0].center));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_2", &rc_handle->rc_channels_range[1].center));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_3", &rc_handle->rc_channels_range[2].center));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_4", &rc_handle->rc_channels_range[3].center));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_5", &rc_handle->rc_channels_range[4].center));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_6", &rc_handle->rc_channels_range[5].center));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_7", &rc_handle->rc_channels_range[6].center));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "RC_CNT_8", &rc_handle->rc_channels_range[7].center));

    // Close
    nvs_close(my_handle);
	return ESP_OK;
}


/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t rc_init(rc_handle_t rc_handle) {
	printf("PPM RX initialization started ... \n");
	memset(rc_handle, 0, sizeof(*rc_handle));

	// set configuration
	rc_handle->config.gpio_num = RC_GPIO_NUM;
	rc_handle->config.channel = RC_CHANNEL;
	rc_handle->config.gpio_num = RC_GPIO_NUM;
	rc_handle->config.clk_div = RC_CLK_DIV;
	rc_handle->config.mem_block_num = RC_BUFF_BLOCK_NUM;
	rc_handle->config.rx_config.filter_ticks_thresh = RC_TICK_TRASH;
	rc_handle->config.rx_config.idle_threshold = RC_PPM_TIMEOUT_US * (RC_TICK_US);
	rc_handle->config.rmt_mode = RMT_MODE_RX;
	rc_handle->config.rx_config.filter_en = true;

	// prepare RMT component
	printf("Gpio Pin [%d] ... \n", rc_handle->config.gpio_num);
	gpio_pulldown_en(rc_handle->config.gpio_num);
	gpio_set_direction(rc_handle->config.gpio_num, GPIO_MODE_INPUT);

	printf("Configuring RMT module ... \n");
	rmt_config(&rc_handle->config);

	printf("Installing RMT module ... \n");
	rmt_driver_install(rc_handle->config.channel, 1000, 0);
	rmt_get_ringbuf_handle(rc_handle->config.channel, &rb);

	printf("RC: load calibration data from NVS\n");
	ESP_ERROR_CHECK(rc_load_calibration_data(rc_handle));
	printf("PPM RX initialized\n");
	return ESP_OK;
}

esp_err_t rc_start(rc_handle_t rc_handle) {
	uint16_t rmt_tick_microseconds = (80 / rc_handle->config.clk_div); /*!< RMT counter value for 10 us.(Source clock is APB clock) */
    rc_handle->data.txrx_signal = RC_TXRX_IGNORE;

	rmt_rx_start(rc_handle->config.channel, true);

	uint8_t counter = 0;
	while (rb) {
		size_t rx_size = 0;
		rmt_item32_t *item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size,
				pdMS_TO_TICKS(25));
		if (item) {
			counter++;
			for (int i = 0; i < RC_MAX_CHANNELS; i++) {
				rc_handle->data.raw[i] = (uint16_t)(((item + i)->duration1
						+ (item + i)->duration0) / rmt_tick_microseconds);
			}
            rc_handle->data.txrx_signal = RC_TXRX_TRANSMITTED;
			vRingbufferReturnItem(rb, (void*) item);
			if(counter == 4) {
				break;
			}
		} else {
			// printf("RC::Data NOT received\n");
		}
	}
	ESP_ERROR_CHECK(rc_stop(rc_handle));
	return ESP_OK;
}

esp_err_t rc_stop(rc_handle_t rc_handle) {
	return rmt_rx_stop(rc_handle->config.channel);
}
