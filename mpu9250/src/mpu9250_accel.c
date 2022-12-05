/*
 * mpu9250_accel.c
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mpu9250_spi.h>
#include <mpu9250_accel.h>
#include <math.h>
#include "esp_system.h"
#include <nvs_flash.h>
#include <nvs.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

static esp_err_t mpu9250_acc_save_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
	uint8_t acc_conf_req = mpu9250_handle->data.accel.fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    uint8_t toWrite = (acc_conf & MPU9250_ACC_FSR_MASK) | ((mpu9250_handle->data.accel.fsr << MPU9250_ACC_FSR_LBIT)& (~MPU9250_ACC_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_acc_load_fsr(mpu9250_handle));

	switch(mpu9250_handle->data.accel.fsr) {
	case INV_FSR_2G: {
		printf("MPU9250: AccFSR 2g\n");
		break;
	}
	case INV_FSR_4G: {
		printf("MPU9250: AccFSR 4g\n");
		break;
	}
	case INV_FSR_8G: {
		printf("MPU9250: AccFSR 8g\n");
		break;
	}
	case INV_FSR_16G: {
		printf("MPU9250: AccFSR 16g\n");
		break;
	}
	default: {
		printf("MPU9250: AccFSR UNKNOWN [%d]\n", mpu9250_handle->data.accel.fsr);
		break;
	}
	}
    return (mpu9250_handle->data.accel.fsr == acc_conf_req ? ESP_OK : ESP_FAIL);
}

static esp_err_t mpu9250_acc_calc_lsb(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->data.accel.lsb = (32768 >> (mpu9250_handle->data.accel.fsr + 1));
	return ESP_OK;
}

static esp_err_t mpu9250_acc_save_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->data.accel.cal.offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->data.accel.cal.offset.xyz.x & 0x00FF;
	buff[2] = 0;
	buff[3] = mpu9250_handle->data.accel.cal.offset.xyz.y >> 8;
	buff[4] = mpu9250_handle->data.accel.cal.offset.xyz.y & 0x00FF;
	buff[5] = 0;
	buff[6] = mpu9250_handle->data.accel.cal.offset.xyz.z >> 8;
	buff[7] = mpu9250_handle->data.accel.cal.offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	return ret;
}

static esp_err_t mpu9250_acc_load_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XA_OFFSET_H, buff, 8*8);
	mpu9250_handle->data.accel.cal.offset.xyz.x = (buff[0] << 8) + buff[1];
	mpu9250_handle->data.accel.cal.offset.xyz.y = (buff[3] << 8) + buff[4];
	mpu9250_handle->data.accel.cal.offset.xyz.z = (buff[6] << 8) + buff[7];
	return ret;
}
/*
# Accelerometer:
> prova_acc$offset
         [,1]
[1,] 118.5426
[2,]  38.3002
[3,] 178.5065

> prova_acc$invA
              [,1]         [,2]          [,3]
[1,]  2.435709e-04 2.391025e-06 -7.584315e-08
[2,]  2.391025e-06 2.442745e-04  1.288202e-06
[3,] -7.584315e-08 1.288202e-06  2.418650e-04

> prova_acc$scale_factors
          [,1]      [,2]      [,3]
[1,] 0.9986014 0.0000000 0.0000000
[2,] 0.0000000 0.9986014 0.0000000
[3,] 0.0000000 0.0000000 0.9986014

> prova_acc$scale_factors %*% prova_acc$invA
              [,1]         [,2]          [,3]
[1,]  2.432302e-04 2.387681e-06 -7.573707e-08
[2,]  2.387681e-06 2.439328e-04  1.286401e-06
[3,] -7.573707e-08 1.286401e-06  2.415267e-04
 */
static esp_err_t mpu9250_acc_set_calibration_data(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_load_offset(mpu9250_handle));
#ifdef CONFIG_ESP_DATA_CAL
	printf("Accel offsets pre: [%d, %d, %d]\n", mpu9250_handle->data.accel.cal.offset.xyz.x, mpu9250_handle->data.accel.cal.offset.xyz.y, mpu9250_handle->data.accel.cal.offset.xyz.z);
//	mpu9250_handle->data.accel.cal.offset.xyz.x += 110;
//	mpu9250_handle->data.accel.cal.offset.xyz.y += 20;
//	mpu9250_handle->data.accel.cal.offset.xyz.z += 159;
//	Accel offsets post: [-6458, 6230, 9227]
	mpu9250_handle->data.accel.cal.offset.xyz.x = -6458;
	mpu9250_handle->data.accel.cal.offset.xyz.y = 6230;
	mpu9250_handle->data.accel.cal.offset.xyz.z = 9227;
	printf("Accel offsets post: [%d, %d, %d]\n", mpu9250_handle->data.accel.cal.offset.xyz.x, mpu9250_handle->data.accel.cal.offset.xyz.y, mpu9250_handle->data.accel.cal.offset.xyz.z);
	ESP_ERROR_CHECK(mpu9250_acc_save_offset(mpu9250_handle));
	mpu9250_handle->data.accel.acc_factors[0] = 2.411398e-04f;
	mpu9250_handle->data.accel.acc_factors[1] = 2.423258e-04f;
	mpu9250_handle->data.accel.acc_factors[2] = 2.398786e-04f;
#else
	mpu9250_handle->data.accel.acc_factors[0] = 1.0f;
	mpu9250_handle->data.accel.acc_factors[1] = 1.0f;
	mpu9250_handle->data.accel.acc_factors[2] = 1.0f;
#endif
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_acc_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->data.accel.rpy.xyz.x = 0;
	mpu9250_handle->data.accel.rpy.xyz.y = 0;
	mpu9250_handle->data.accel.rpy.xyz.z = 0;
	mpu9250_handle->data.accel.acc_g_factor = 1.0f;
	ESP_ERROR_CHECK(mpu9250_acc_set_calibration_data(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_load_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t acc_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_ACCEL_CONFIG, &acc_conf));
    mpu9250_handle->data.accel.fsr = (acc_conf & (~MPU9250_ACC_FSR_MASK)) >> MPU9250_ACC_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_acc_calc_lsb(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr) {
	mpu9250_handle->data.accel.fsr=fsr;
	ESP_ERROR_CHECK(mpu9250_acc_save_fsr(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_acc_load_cal_data(mpu9250_handle_t mpu9250_handle) {
#ifdef CONFIG_ESP_DATA_CAL
	mpu9250_handle->data.cal_data.data_s_xyz.accel_data_x = mpu9250_handle->data.accel.acc_factors[0]*mpu9250_handle->data.raw_data.data_s_xyz.accel_data_x;
	mpu9250_handle->data.cal_data.data_s_xyz.accel_data_y = mpu9250_handle->data.accel.acc_factors[1]*mpu9250_handle->data.raw_data.data_s_xyz.accel_data_y;
	mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z = mpu9250_handle->data.accel.acc_factors[2]*mpu9250_handle->data.raw_data.data_s_xyz.accel_data_z;
#endif
	return ESP_OK;
}
