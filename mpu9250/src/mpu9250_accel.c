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
#include "esp_log.h"
#include <nvs_flash.h>
#include <nvs.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
static const char *TAG = "my_mpu9250_acc";

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


static esp_err_t mpu9250_acc_load_default_calibration_params(
		mpu9250_handle_t mpu9250_handle) {

	mpu9250_handle->data.accel.cal.factors[X_POS][X_POS] = 1.0f;
	mpu9250_handle->data.accel.cal.factors[X_POS][Y_POS] = 0.0f;
	mpu9250_handle->data.accel.cal.factors[X_POS][Z_POS] = 0.0f;
	mpu9250_handle->data.accel.cal.factors[Y_POS][X_POS] = 0.0f;
	mpu9250_handle->data.accel.cal.factors[Y_POS][Y_POS] = 1.0f;
	mpu9250_handle->data.accel.cal.factors[Y_POS][Z_POS] = 0.0f;
	mpu9250_handle->data.accel.cal.factors[Z_POS][X_POS] = 0.0f;
	mpu9250_handle->data.accel.cal.factors[Z_POS][Y_POS] = 0.0f;
	mpu9250_handle->data.accel.cal.factors[Z_POS][Z_POS] = 1.0f;
	mpu9250_handle->data.accel.cal.offsets[X_POS] = 0.0;
	mpu9250_handle->data.accel.cal.offsets[Y_POS] = 0.0;
	mpu9250_handle->data.accel.cal.offsets[Z_POS] = 0.0;
	return ESP_OK;
}

esp_err_t mpu9250_acc_load_calibration_params(
		mpu9250_handle_t mpu9250_handle) {
	nvs_handle_t nvs_handle = (nvs_handle_t)mpu9250_handle->data.nvs_cal_data;

    // Lettura del blob di 48 byte dalla memoria flash
    uint8_t blob[48];
    size_t blob_size = sizeof(blob);
    esp_err_t err = nvs_get_blob(nvs_handle, "acc_model", blob, &blob_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Acc Calibration Params are not flashed: %s\n", esp_err_to_name(err));
        ESP_ERROR_CHECK(mpu9250_acc_load_default_calibration_params(mpu9250_handle));
        return ESP_OK;
    }

    // Controllo della dimensione del blob
    if (blob_size != 48) {
    	ESP_LOGW(TAG, "Acc params: Dimensione del blob non corretta (%d). I load default params ...", blob_size);
        ESP_ERROR_CHECK(mpu9250_acc_load_default_calibration_params(mpu9250_handle));
        return ESP_OK;
    }

    // Conversione del blob in una matrice float 3x3
    memcpy(mpu9250_handle->data.accel.cal.factors, blob, 36);
    memcpy(mpu9250_handle->data.accel.cal.offsets, (blob+36), 12);
    ESP_LOGI(TAG, "Acc matrix:\n[[%5.5f,%5.5f,%5.5f]\n,[%5.5f,%5.5f,%5.5f]\n,[%5.5f,%5.5f,%5.5f]]\n",
    		mpu9250_handle->data.accel.cal.factors[X_POS][X_POS],
    		mpu9250_handle->data.accel.cal.factors[X_POS][Y_POS],
    		mpu9250_handle->data.accel.cal.factors[X_POS][Z_POS],
    		mpu9250_handle->data.accel.cal.factors[Y_POS][X_POS],
    		mpu9250_handle->data.accel.cal.factors[Y_POS][Y_POS],
    		mpu9250_handle->data.accel.cal.factors[Y_POS][Z_POS],
    		mpu9250_handle->data.accel.cal.factors[Z_POS][X_POS],
    		mpu9250_handle->data.accel.cal.factors[Z_POS][Y_POS],
    		mpu9250_handle->data.accel.cal.factors[Z_POS][Z_POS]
    		);
    ESP_LOGI(TAG, "Acc bias:\n [%5.5f,%5.5f,%5.5f]\n",
	   		mpu9250_handle->data.accel.cal.offsets[X_POS],
	   		mpu9250_handle->data.accel.cal.offsets[Y_POS],
	   		mpu9250_handle->data.accel.cal.offsets[Z_POS]
    		);
	return ESP_OK;
}

esp_err_t mpu9250_acc_save_calibration_params(mpu9250_handle_t mpu9250_handle, char* data, int data_len) {
    if (data_len == 48) {  // 3x3 matrix + 3x1 bias = 12 floats = 12 * sizeof(float) = 12x4 = 48 bytes
        // Save the matrix to flash
        ESP_ERROR_CHECK(nvs_set_blob((nvs_handle_t)mpu9250_handle->data.nvs_cal_data, "acc_model", data, data_len));
        ESP_ERROR_CHECK(nvs_commit((nvs_handle_t)mpu9250_handle->data.nvs_cal_data));
        ESP_LOGI(TAG, "Acc matrix+bias saved to flash");
    } else {
        ESP_LOGE(TAG, "Acc Invalid matrix+bias data length");
    }
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_acc_init(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_acc_load_calibration_params(mpu9250_handle));
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
	mpu9250_handle->data.cal_data.data_s_xyz.accel_data_x = mpu9250_handle->data.accel.cal.factors[X_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_x - mpu9250_handle->data.accel.cal.offsets[X_POS]) + mpu9250_handle->data.accel.cal.factors[X_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_y - mpu9250_handle->data.accel.cal.offsets[Y_POS]) + mpu9250_handle->data.accel.cal.factors[X_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_z - mpu9250_handle->data.accel.cal.offsets[Z_POS]);
	mpu9250_handle->data.cal_data.data_s_xyz.accel_data_y = mpu9250_handle->data.accel.cal.factors[Y_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_x - mpu9250_handle->data.accel.cal.offsets[X_POS]) + mpu9250_handle->data.accel.cal.factors[Y_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_y - mpu9250_handle->data.accel.cal.offsets[Y_POS]) + mpu9250_handle->data.accel.cal.factors[Y_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_z - mpu9250_handle->data.accel.cal.offsets[Z_POS]);
	mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z = mpu9250_handle->data.accel.cal.factors[Z_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_x - mpu9250_handle->data.accel.cal.offsets[X_POS]) + mpu9250_handle->data.accel.cal.factors[Z_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_y - mpu9250_handle->data.accel.cal.offsets[Y_POS]) + mpu9250_handle->data.accel.cal.factors[Z_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.accel_data_z - mpu9250_handle->data.accel.cal.offsets[Z_POS]);
	return ESP_OK;
}
