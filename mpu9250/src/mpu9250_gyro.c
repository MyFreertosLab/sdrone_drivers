/*
 * mpu9250_gyro.c
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mpu9250_spi.h>
#include <mpu9250_gyro.h>
#include <math.h>
#include "esp_system.h"
#include <nvs_flash.h>
#include <nvs.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
static esp_err_t mpu9250_gyro_load_calibration_data(mpu9250_handle_t mpu9250_handle) {
    nvs_handle_t my_handle;
    uint8_t flashed = 0;
    ESP_ERROR_CHECK(nvs_open("GYRO_CAL", NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_get_u8(my_handle, "FLASHED", &flashed));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_OFF", &mpu9250_handle->data.gyro.cal.offset.array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_OFF", &mpu9250_handle->data.gyro.cal.offset.array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_OFF", &mpu9250_handle->data.gyro.cal.offset.array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_250", &mpu9250_handle->data.gyro.cal.var[INV_FSR_250DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_250", &mpu9250_handle->data.gyro.cal.var[INV_FSR_250DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_250", &mpu9250_handle->data.gyro.cal.var[INV_FSR_250DPS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_500", &mpu9250_handle->data.gyro.cal.var[INV_FSR_500DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_500", &mpu9250_handle->data.gyro.cal.var[INV_FSR_500DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_500", &mpu9250_handle->data.gyro.cal.var[INV_FSR_500DPS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_1000", &mpu9250_handle->data.gyro.cal.var[INV_FSR_1000DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_1000", &mpu9250_handle->data.gyro.cal.var[INV_FSR_1000DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_1000", &mpu9250_handle->data.gyro.cal.var[INV_FSR_1000DPS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "X_VAR_2000", &mpu9250_handle->data.gyro.cal.var[INV_FSR_2000DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Y_VAR_2000", &mpu9250_handle->data.gyro.cal.var[INV_FSR_2000DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_u16(my_handle, "Z_VAR_2000", &mpu9250_handle->data.gyro.cal.var[INV_FSR_2000DPS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_250", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_250DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_250", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_250DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_250", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_250DPS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_500", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_500DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_500", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_500DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_500", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_500DPS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_1000", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_1000DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_1000", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_1000DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_1000", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_1000DPS].array[Z_POS]));

    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "X_SQM_2000", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_2000DPS].array[X_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Y_SQM_2000", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_2000DPS].array[Y_POS]));
    ESP_ERROR_CHECK(nvs_get_i16(my_handle, "Z_SQM_2000", &mpu9250_handle->data.gyro.cal.sqm[INV_FSR_2000DPS].array[Z_POS]));
	printf("Gyro: loaded calibration data from NVS \n");
    printf("Gyro offsets [%d][%d][%d]:\n", mpu9250_handle->data.gyro.cal.offset.array[X_POS], mpu9250_handle->data.gyro.cal.offset.array[Y_POS],mpu9250_handle->data.gyro.cal.offset.array[Z_POS]);
    printf("Gyro var [%d][%d][%d]:\n",
    		mpu9250_handle->data.gyro.cal.var[INV_FSR_250DPS].array[X_POS],
			mpu9250_handle->data.gyro.cal.var[INV_FSR_250DPS].array[Y_POS],
			mpu9250_handle->data.gyro.cal.var[INV_FSR_250DPS].array[Z_POS]);
    printf("Gyro sqm [%d][%d][%d]:\n",
    		mpu9250_handle->data.gyro.cal.sqm[INV_FSR_250DPS].array[X_POS],
			mpu9250_handle->data.gyro.cal.sqm[INV_FSR_250DPS].array[Y_POS],
			mpu9250_handle->data.gyro.cal.sqm[INV_FSR_250DPS].array[Z_POS]);

    // Close
    nvs_close(my_handle);

	return ESP_OK;
}

static esp_err_t mpu9250_gyro_save_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t gyro_conf = 0;
	uint8_t gyro_conf_req = mpu9250_handle->data.gyro.fsr;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_GYRO_CONFIG, &gyro_conf));
    uint8_t toWrite = (gyro_conf & MPU9250_GYRO_FSR_MASK) | ((mpu9250_handle->data.gyro.fsr << MPU9250_GYRO_FSR_LBIT)& (~MPU9250_GYRO_FSR_MASK));
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_GYRO_CONFIG, toWrite));
    ESP_ERROR_CHECK(mpu9250_gyro_load_fsr(mpu9250_handle));

	switch(mpu9250_handle->data.gyro.fsr) {
	case INV_FSR_250DPS: {
		printf("MPU9250: GyroFSR 250dps\n");
		break;
	}
	case INV_FSR_500DPS: {
		printf("MPU9250: GyroFSR 500dps\n");
		break;
	}
	case INV_FSR_1000DPS: {
		printf("MPU9250: GyroFSR 1000dps\n");
		break;
	}
	case INV_FSR_2000DPS: {
		printf("MPU9250: GyroFSR 2000dps\n");
		break;
	}
	default: {
		printf("MPU9250: GyroFSR UNKNOWN [%d]\n", mpu9250_handle->data.gyro.fsr);
		break;
	}
	}
    return (mpu9250_handle->data.gyro.fsr == gyro_conf_req ? ESP_OK : ESP_FAIL);
}

static esp_err_t mpu9250_gyro_init_kalman_filter(mpu9250_handle_t mpu9250_handle) {
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->data.gyro.cal.kalman[i].X = mpu9250_handle->data.gyro.lsb;
		mpu9250_handle->data.gyro.cal.kalman[i].sample=0;
		mpu9250_handle->data.gyro.cal.kalman[i].P=1.0f;
		mpu9250_handle->data.gyro.cal.kalman[i].Q=1.5;
		mpu9250_handle->data.gyro.cal.kalman[i].K=0.0f;
		mpu9250_handle->data.gyro.cal.kalman[i].R=mpu9250_handle->data.gyro.cal.var[mpu9250_handle->data.gyro.fsr].array[i];
		mpu9250_handle->data.gyro.cal.kalman[i].initialized = 0;
	}
	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_lsb(mpu9250_handle_t mpu9250_handle) {
	switch(mpu9250_handle->data.gyro.fsr) {
	case(INV_FSR_2000DPS): {
		mpu9250_handle->data.gyro.lsb = 16.384;
		break;
	}
	case(INV_FSR_1000DPS): {
		mpu9250_handle->data.gyro.lsb = 32.768;
		break;
	}
	case(INV_FSR_500DPS): {
		mpu9250_handle->data.gyro.lsb = 65.536;
		break;
	}
	case(INV_FSR_250DPS): {
		mpu9250_handle->data.gyro.lsb = 131.072;
		break;
	}
	}
	return ESP_OK;
}


static esp_err_t mpu9250_gyro_save_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[8];
	buff[0] = mpu9250_handle->data.gyro.cal.offset.xyz.x >> 8;
	buff[1] = mpu9250_handle->data.gyro.cal.offset.xyz.x & 0x00FF;
	buff[2] = mpu9250_handle->data.gyro.cal.offset.xyz.y >> 8;
	buff[3] = mpu9250_handle->data.gyro.cal.offset.xyz.y & 0x00FF;
	buff[4] = mpu9250_handle->data.gyro.cal.offset.xyz.z >> 8;
	buff[5] = mpu9250_handle->data.gyro.cal.offset.xyz.z & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	return ret;
}

static esp_err_t mpu9250_gyro_filter_data(mpu9250_handle_t mpu9250_handle) {
	int16_t alfa_mean;
	for(uint8_t i = 0; i < 3; i++) {
		if(mpu9250_handle->data.gyro.cal.kalman[i].P > 0.01) {
			mpu9250_cb_last(&mpu9250_handle->data.gyro.cb[i], &mpu9250_handle->data.gyro.cal.kalman[i].sample);
			if(mpu9250_handle->data.gyro.cal.kalman[i].initialized == 0) {
				mpu9250_handle->data.gyro.cal.kalman[i].X = mpu9250_handle->data.gyro.cal.kalman[i].sample;
				mpu9250_handle->data.gyro.cal.kalman[i].initialized = 1;
			}
			int16_t prevX = mpu9250_handle->data.gyro.cal.kalman[i].X;
			mpu9250_handle->data.gyro.cal.kalman[i].P = mpu9250_handle->data.gyro.cal.kalman[i].P+mpu9250_handle->data.gyro.cal.kalman[i].Q;
			mpu9250_handle->data.gyro.cal.kalman[i].K = mpu9250_handle->data.gyro.cal.kalman[i].P/(mpu9250_handle->data.gyro.cal.kalman[i].P+mpu9250_handle->data.gyro.cal.kalman[i].R);
			mpu9250_handle->data.gyro.cal.kalman[i].X = mpu9250_handle->data.gyro.cal.kalman[i].X + mpu9250_handle->data.gyro.cal.kalman[i].K*(mpu9250_handle->data.gyro.cal.kalman[i].sample - mpu9250_handle->data.gyro.cal.kalman[i].X);
			mpu9250_handle->data.gyro.cal.kalman[i].P=(1-mpu9250_handle->data.gyro.cal.kalman[i].K)*mpu9250_handle->data.gyro.cal.kalman[i].P;
			// in rad/sec/sec
			mpu9250_cb_add(&mpu9250_handle->data.gyro.cb_alfa[i], mpu9250_handle->data.gyro.cal.kalman[i].X - prevX);
			mpu9250_cb_means(&mpu9250_handle->data.gyro.cb_alfa[i], &alfa_mean);
			mpu9250_handle->data.gyro.alfa[i] = (double)alfa_mean*(mpu9250_handle->data_rate/5.0f)/(double)mpu9250_handle->data.gyro.lsb/(double)360.0f*(double)PI_2;
		}
	}
	return ESP_OK;
}

static esp_err_t mpu9250_gyro_calc_rpy(mpu9250_handle_t mpu9250_handle) {
	// angolo di rotazione: w(i)=domega(i)*dt espresso in rad
	double w[3] = {0.0f,0.0f,0.0f};
	for(uint8_t i = 0; i < 3; i++) {
		w[i] = (double)(mpu9250_handle->data.gyro.cal.kalman[i].X)/(double)mpu9250_handle->data.gyro.lsb/(double)mpu9250_handle->data_rate/(double)360.0f*(double)PI_2;
	}

	mpu9250_handle->data.gyro.rpy.xyz.x += w[X_POS];
	mpu9250_handle->data.gyro.rpy.xyz.y += w[Y_POS];
	mpu9250_handle->data.gyro.rpy.xyz.z += w[Z_POS];

	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_gyro_init(mpu9250_handle_t mpu9250_handle) {
	mpu9250_gyro_load_calibration_data(mpu9250_handle); // set offset, var, sqm, P, K
	mpu9250_gyro_save_offset(mpu9250_handle);
	mpu9250_gyro_init_kalman_filter(mpu9250_handle); // this reset P,K
	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_handle->data.gyro.rpy.array[i] = 0;
		mpu9250_handle->data.gyro.alfa[i] = 0.0f;
	}
	return ESP_OK;
}

esp_err_t mpu9250_gyro_load_fsr(mpu9250_handle_t mpu9250_handle) {
	uint8_t gyro_conf = 0;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_GYRO_CONFIG, &gyro_conf));
    mpu9250_handle->data.gyro.fsr = (gyro_conf & (~MPU9250_GYRO_FSR_MASK)) >> MPU9250_GYRO_FSR_LBIT;
    ESP_ERROR_CHECK(mpu9250_gyro_calc_lsb(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_gyro_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr) {
	mpu9250_handle->data.gyro.fsr=fsr;
	ESP_ERROR_CHECK(mpu9250_gyro_save_fsr(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_gyro_load_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[6];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	mpu9250_handle->data.gyro.cal.offset.xyz.x = ((buff[0] << 8) + buff[1]);
	mpu9250_handle->data.gyro.cal.offset.xyz.y = ((buff[2] << 8) + buff[3]);
	mpu9250_handle->data.gyro.cal.offset.xyz.z = ((buff[4] << 8) + buff[5]);
	return ret;
}
esp_err_t mpu9250_gyro_set_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff) {
	mpu9250_handle->data.gyro.cal.offset.xyz.x = xoff;
	mpu9250_handle->data.gyro.cal.offset.xyz.y = yoff;
	mpu9250_handle->data.gyro.cal.offset.xyz.z = zoff;
	return mpu9250_gyro_save_offset(mpu9250_handle);
}

esp_err_t mpu9250_gyro_update_state(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_gyro_filter_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_gyro_calc_rpy(mpu9250_handle));
	return ESP_OK;
}

