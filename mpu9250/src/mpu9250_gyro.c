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
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
static const char *TAG = "my_mpu9250_gyro";

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
	buff[0] = mpu9250_handle->data.gyro.device_offsets[X_POS] >> 8;
	buff[1] = mpu9250_handle->data.gyro.device_offsets[X_POS] & 0x00FF;
	buff[2] = mpu9250_handle->data.gyro.device_offsets[Y_POS] >> 8;
	buff[3] = mpu9250_handle->data.gyro.device_offsets[Y_POS] & 0x00FF;
	buff[4] = mpu9250_handle->data.gyro.device_offsets[Z_POS] >> 8;
	buff[5] = mpu9250_handle->data.gyro.device_offsets[Z_POS] & 0x00FF;

	esp_err_t ret = mpu9250_write_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	return ret;
}


static esp_err_t mpu9250_gyro_load_default_calibration_params(
		mpu9250_handle_t mpu9250_handle) {

	mpu9250_handle->data.gyro.cal.factors[X_POS][X_POS] = 1.0f;
	mpu9250_handle->data.gyro.cal.factors[X_POS][Y_POS] = 0.0f;
	mpu9250_handle->data.gyro.cal.factors[X_POS][Z_POS] = 0.0f;
	mpu9250_handle->data.gyro.cal.factors[Y_POS][X_POS] = 0.0f;
	mpu9250_handle->data.gyro.cal.factors[Y_POS][Y_POS] = 1.0f;
	mpu9250_handle->data.gyro.cal.factors[Y_POS][Z_POS] = 0.0f;
	mpu9250_handle->data.gyro.cal.factors[Z_POS][X_POS] = 0.0f;
	mpu9250_handle->data.gyro.cal.factors[Z_POS][Y_POS] = 0.0f;
	mpu9250_handle->data.gyro.cal.factors[Z_POS][Z_POS] = 1.0f;
	mpu9250_handle->data.gyro.cal.offsets[X_POS] = 0.0;
	mpu9250_handle->data.gyro.cal.offsets[Y_POS] = 0.0;
	mpu9250_handle->data.gyro.cal.offsets[Z_POS] = 0.0;
	return ESP_OK;
}

esp_err_t mpu9250_gyro_load_calibration_params(
		mpu9250_handle_t mpu9250_handle) {
	nvs_handle_t nvs_handle = (nvs_handle_t)mpu9250_handle->data.nvs_cal_data;

    // Lettura del blob di 48 byte dalla memoria flash
    uint8_t blob[48];
    size_t blob_size = sizeof(blob);
    esp_err_t err = nvs_get_blob(nvs_handle, "gyro_model", blob, &blob_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Gyro Calibration Params are not flashed: %s\n", esp_err_to_name(err));
        ESP_ERROR_CHECK(mpu9250_gyro_load_default_calibration_params(mpu9250_handle));
        return ESP_OK;
    }

    // Controllo della dimensione del blob
    if (blob_size != 48) {
    	ESP_LOGW(TAG, "Gyro params: Dimensione del blob non corretta (%d). I load default params ...", blob_size);
        ESP_ERROR_CHECK(mpu9250_gyro_load_default_calibration_params(mpu9250_handle));
        return ESP_OK;
    }

    // Conversione del blob in una matrice float 3x3
    memcpy(mpu9250_handle->data.gyro.cal.factors, blob, 36);
    memcpy(mpu9250_handle->data.gyro.cal.offsets, (blob+36), 12);
    ESP_LOGI(TAG, "Gyro matrix:\n[[%5.5f,%5.5f,%5.5f]\n,[%5.5f,%5.5f,%5.5f]\n,[%5.5f,%5.5f,%5.5f]]\n",
    		mpu9250_handle->data.gyro.cal.factors[X_POS][X_POS],
    		mpu9250_handle->data.gyro.cal.factors[X_POS][Y_POS],
    		mpu9250_handle->data.gyro.cal.factors[X_POS][Z_POS],
    		mpu9250_handle->data.gyro.cal.factors[Y_POS][X_POS],
    		mpu9250_handle->data.gyro.cal.factors[Y_POS][Y_POS],
    		mpu9250_handle->data.gyro.cal.factors[Y_POS][Z_POS],
    		mpu9250_handle->data.gyro.cal.factors[Z_POS][X_POS],
    		mpu9250_handle->data.gyro.cal.factors[Z_POS][Y_POS],
    		mpu9250_handle->data.gyro.cal.factors[Z_POS][Z_POS]
    		);
    ESP_LOGI(TAG, "Gyro bias:\n [%5.5f,%5.5f,%5.5f]\n",
	   		mpu9250_handle->data.gyro.cal.offsets[X_POS],
	   		mpu9250_handle->data.gyro.cal.offsets[Y_POS],
	   		mpu9250_handle->data.gyro.cal.offsets[Z_POS]
    		);
	return ESP_OK;
}

esp_err_t mpu9250_gyro_save_calibration_params(mpu9250_handle_t mpu9250_handle, char* data, int data_len) {
    if (data_len == 48) {  // 3x3 matrix + 3x1 bias = 12 floats = 12 * sizeof(float) = 12x4 = 48 bytes
        // Save the matrix to flash
        ESP_ERROR_CHECK(nvs_set_blob((nvs_handle_t)mpu9250_handle->data.nvs_cal_data, "gyro_model", data, data_len));
        ESP_ERROR_CHECK(nvs_commit((nvs_handle_t)mpu9250_handle->data.nvs_cal_data));
        ESP_LOGI(TAG, "Gyro matrix+bias saved to flash");
    } else {
        ESP_LOGE(TAG, "Gyro Invalid matrix+bias data length");
    }
	return ESP_OK;
}



/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_gyro_init(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_gyro_load_calibration_params(mpu9250_handle));
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

esp_err_t mpu9250_gyro_load_device_offset(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[6];
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_XG_OFFSET_H, buff, 6*8);
	mpu9250_handle->data.gyro.device_offsets[X_POS] = ((buff[0] << 8) + buff[1]);
	mpu9250_handle->data.gyro.device_offsets[Y_POS] = ((buff[2] << 8) + buff[3]);
	mpu9250_handle->data.gyro.device_offsets[Z_POS] = ((buff[4] << 8) + buff[5]);
	return ret;
}

esp_err_t mpu9250_gyro_set_device_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff) {
	mpu9250_handle->data.gyro.device_offsets[X_POS] = xoff;
	mpu9250_handle->data.gyro.device_offsets[Y_POS] = yoff;
	mpu9250_handle->data.gyro.device_offsets[Z_POS] = zoff;
	return mpu9250_gyro_save_offset(mpu9250_handle);
}

esp_err_t mpu9250_gyro_load_cal_data(mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->data.cal_data.data_s_xyz.gyro_data_x = mpu9250_handle->data.gyro.cal.factors[X_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_x - mpu9250_handle->data.gyro.cal.offsets[X_POS]) + mpu9250_handle->data.gyro.cal.factors[X_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_y - mpu9250_handle->data.gyro.cal.offsets[Y_POS]) + mpu9250_handle->data.gyro.cal.factors[X_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_z - mpu9250_handle->data.gyro.cal.offsets[Z_POS]);
	mpu9250_handle->data.cal_data.data_s_xyz.gyro_data_y = mpu9250_handle->data.gyro.cal.factors[Y_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_x - mpu9250_handle->data.gyro.cal.offsets[X_POS]) + mpu9250_handle->data.gyro.cal.factors[Y_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_y - mpu9250_handle->data.gyro.cal.offsets[Y_POS]) + mpu9250_handle->data.gyro.cal.factors[Y_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_z - mpu9250_handle->data.gyro.cal.offsets[Z_POS]);
	mpu9250_handle->data.cal_data.data_s_xyz.gyro_data_x = mpu9250_handle->data.gyro.cal.factors[Z_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_x - mpu9250_handle->data.gyro.cal.offsets[X_POS]) + mpu9250_handle->data.gyro.cal.factors[Z_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_y - mpu9250_handle->data.gyro.cal.offsets[Y_POS]) + mpu9250_handle->data.gyro.cal.factors[Z_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_z - mpu9250_handle->data.gyro.cal.offsets[Z_POS]);
	return ESP_OK;
}
