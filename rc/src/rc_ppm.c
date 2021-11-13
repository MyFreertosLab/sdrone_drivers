/*
 * ppm8.c
 *
 *  Created on: 23 feb 2021
 *      Author: andrea
 */
#include <string.h>
#include <rc_ppm.h>
#include <driver/gpio.h>
#include "driver/rmt.h"
#include <esp_system.h>

#define RC_CHANNEL            0                  /* Channel input (0-7) for receiver */
#define RC_GPIO_NUM           23                 /* GPIO number for receiver */
#define RC_CLK_DIV            8                  /* Clock divider */
#define RC_TICK_US            (80/RC_CLK_DIV)    /* Number of Ticks for us */
#define RC_PPM_TIMEOUT_US     3500               /* min PPM silence (us) */
#define RC_BUFF_BLOCK_NUM     4                  /* Memory Blocks */
#define RC_TICK_TRASH         100                /* Interference */


#define RC_PPM_DEVICE_CONFIG(rc_handle) ((rc_ppm_config_t*)rc_handle->dev_config)


typedef struct {
	rmt_config_t rmt_config;
} rc_ppm_config_t;

static rc_ppm_config_t rc_ppm_config = {
		.rmt_config = {
				  .channel = RC_CHANNEL,
				  .gpio_num = RC_GPIO_NUM,
				  .clk_div = RC_CLK_DIV,
				  .mem_block_num = RC_BUFF_BLOCK_NUM,
				  .rmt_mode = RMT_MODE_RX,
				  .rx_config.filter_en = true,
				  .rx_config.filter_ticks_thresh = RC_TICK_TRASH,
				  .rx_config.idle_threshold = RC_PPM_TIMEOUT_US * (RC_TICK_US)
				 }
};

//FIXME: transfer this in rc_t
static RingbufHandle_t rb = NULL;

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t rc_ppm_init(rc_handle_t rc_handle) {
	memset(rc_handle, 0, sizeof(*rc_handle));
	rc_handle->dev_config = (void*)&rc_ppm_config;
	printf("PPM RX initialization started ... \n");
	printf("Gpio Pin [%d] ... \n", rc_ppm_config.rmt_config.gpio_num);
	gpio_pulldown_en(rc_ppm_config.rmt_config.gpio_num);
	gpio_set_direction(rc_ppm_config.rmt_config.gpio_num, GPIO_MODE_INPUT);

	printf("Configuring RMT module ... \n");
	rmt_config(&rc_ppm_config.rmt_config);

	printf("Installing RMT module ... \n");
	rmt_driver_install(rc_ppm_config.rmt_config.channel, 1000, 0);


	printf("Configure stick to channel map ... \n");
	for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
		rc_handle->rc_channels_range[i].min = 0;
		rc_handle->rc_channels_range[i].max = 0;
		rc_handle->rc_channels_range[i].center = 0;
	}

	int channel = RC_PPM_DEVICE_CONFIG(rc_handle)->rmt_config.channel;
	rmt_get_ringbuf_handle(channel, &rb);
	printf("PPM RX initialized\n");
	return ESP_OK;
}

esp_err_t rc_ppm_start(rc_handle_t rc_handle) {
	uint16_t rmt_tick_microseconds = (80 / RC_PPM_DEVICE_CONFIG(rc_handle)->rmt_config.clk_div); /*!< RMT counter value for 10 us.(Source clock is APB clock) */
	rc_handle->data.txrx_signal = RC_TXRX_IGNORE;
	int channel = RC_PPM_DEVICE_CONFIG(rc_handle)->rmt_config.channel;

	rmt_rx_start(channel, true);

	uint8_t channels = 0;
	uint8_t counter = 0;
	while (rb) {
		size_t rx_size = 0;
		rmt_item32_t *item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size,
				pdMS_TO_TICKS(25));
		if (item) {
			channels = (rx_size / 4) - 1;
			counter++;
			for (int i = 0; i < channels; i++) {
				rc_handle->data.raw[i] = (uint16_t) (((item + i)->duration1
						+ (item + i)->duration0) / rmt_tick_microseconds);
			}
			rc_handle->data.txrx_signal = RC_TXRX_TRANSMITTED;
			vRingbufferReturnItem(rb, (void*) item);
			if (counter == 10) {
				break;
			}
		} else {
			rc_handle->data.txrx_signal = RC_TXRX_DATA_NOT_RECEIVED;
			break;
		}
	}
	if(rc_handle->data.raw[RC_THROTTLE] >= 1500 && rc_handle->data.raw[RC_PITCH] <= 910) {
		rc_handle->data.txrx_signal = RC_TXRX_DATA_NOT_RECEIVED;
	}
	ESP_ERROR_CHECK(rc_ppm_stop(rc_handle));
	return ESP_OK;
}

esp_err_t rc_ppm_stop(rc_handle_t rc_handle) {
	int channel = RC_PPM_DEVICE_CONFIG(rc_handle)->rmt_config.channel;
	return rmt_rx_stop(channel);
}
