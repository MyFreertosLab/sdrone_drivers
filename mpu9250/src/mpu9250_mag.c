/*
 * mpu9250_mag.c
 *
 *  Created on: 25 set 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mpu9250_spi.h>
#include <mpu9250_mag.h>
#include <math.h>
#include <esp_system.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
static const char *TAG = "my_mpu9250_mag";

static esp_err_t WriteAk8963Register(mpu9250_handle_t mpu9250_handle, uint8_t reg,
		uint8_t data) {
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_DO, data));
	uint8_t slv0_en = (I2C_SLV_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv0_en));
	return ESP_OK;
}

static esp_err_t ReadAk8963Register(mpu9250_handle_t mpu9250_handle, uint8_t reg) {
	uint8_t addr = (AK8963_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	uint8_t slv0_en = (I2C_SLV_EN | 0x01);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv0_en));
	return ESP_OK;
}
static esp_err_t ReadAk8963Registers(mpu9250_handle_t mpu9250_handle, uint8_t reg, uint8_t count) {
	uint8_t addr = (AK8963_ADDRESS | MPU9250_I2C_READ_FLAG);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, addr));
	ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, reg));
	uint8_t slv0_en = (I2C_SLV_EN | count);
	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, slv0_en));
	return ESP_OK;
}

static esp_err_t mpu9250_mag_calc_precision_factor(mpu9250_handle_t mpu9250_handle) {
	switch(mpu9250_handle->data.mag.precision) {
	case(INV_MAG_PRECISION_14_BITS): {
		mpu9250_handle->data.mag.precision_factor = 0.6f;
		printf("MPU9250: Mag Precision 0.6 \n");
		break;
	}
	case(INV_MAG_PRECISION_16_BITS): {
		mpu9250_handle->data.mag.precision_factor = 0.15f;
		printf("MPU9250: Mag Precision 0.15 \n");
		break;
	}
	default: {
		printf("MPU9250: Mag Precision UNKNOWN [%d]\n", mpu9250_handle->data.mag.precision);
		return ESP_FAIL;
	}
	}
	return ESP_OK;
}

static esp_err_t mpu9250_mag_save_precision(mpu9250_handle_t mpu9250_handle) {
	uint8_t mag_conf = 0;

	// read cntl1
    ESP_ERROR_CHECK(ReadAk8963Register(mpu9250_handle, AK8963_CNTL1));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &mag_conf));
	printf("MAG SAVE mag_conf=[%d]\n", mag_conf);

	// calc configuration for requested precision
	switch(mpu9250_handle->data.mag.precision) {
	case INV_MAG_PRECISION_14_BITS: {
		mag_conf &= ~(AK8963_PRECISION_MASK);
		printf("MAG SAVE 14BITS mag_conf=[%d]\n", mag_conf);
		break;
	}
	case INV_MAG_PRECISION_16_BITS: {
		mag_conf |= AK8963_PRECISION_MASK;
		printf("MAG SAVE 16BITS mag_conf=[%d]\n", mag_conf);
		break;
	}
	default: {
		printf("MPU9250: Mag Precision UNKNOWN [%d]\n", mpu9250_handle->data.mag.precision);
		return ESP_FAIL;
	}
	}

    ESP_ERROR_CHECK(
    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, mag_conf));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(
    		mpu9250_mag_calc_precision_factor(mpu9250_handle));

	return ESP_OK;
}
/*
# Magnetometer:
prova$als$c
          [,1]
[1,]  55.35344
[2,] 184.20142
[3,] 162.19372

prova$invA
              [,1]          [,2]          [,3]
[1,]  0.0076966141 -0.0001321952 -0.0004309371
[2,] -0.0001321952  0.0078691169  0.0001325589
[3,] -0.0004309371  0.0001325589  0.0087519421

> prova$scale_factors
          [,1]      [,2]      [,3]
[1,] 0.5530303 0.0000000 0.0000000
[2,] 0.0000000 0.5530303 0.0000000
[3,] 0.0000000 0.0000000 0.5530303

> prova$scale_factors%*%prova$invA
              [,1]          [,2]          [,3]
[1,]  4.256460e-03 -7.310792e-05 -2.383212e-04
[2,] -7.310792e-05  4.351860e-03  7.330909e-05
[3,] -2.383212e-04  7.330909e-05  4.840089e-03
 */
static esp_err_t mpu9250_mag_load_default_calibration_params(
		mpu9250_handle_t mpu9250_handle) {
	mpu9250_handle->data.mag.cal.offsets[X_POS] = 0;
	mpu9250_handle->data.mag.cal.offsets[Y_POS] = 0;
	mpu9250_handle->data.mag.cal.offsets[Z_POS] = 0;

	mpu9250_handle->data.mag.cal.factors[X_POS][X_POS] = 1.0f;
	mpu9250_handle->data.mag.cal.factors[X_POS][Y_POS] = 0.0f;
	mpu9250_handle->data.mag.cal.factors[X_POS][Z_POS] = 0.0f;
	mpu9250_handle->data.mag.cal.factors[Y_POS][X_POS] = 0.0f;
	mpu9250_handle->data.mag.cal.factors[Y_POS][Y_POS] = 1.0f;
	mpu9250_handle->data.mag.cal.factors[Y_POS][Z_POS] = 0.0f;
	mpu9250_handle->data.mag.cal.factors[Z_POS][X_POS] = 0.0f;
	mpu9250_handle->data.mag.cal.factors[Z_POS][Y_POS] = 0.0f;
	mpu9250_handle->data.mag.cal.factors[Z_POS][Z_POS] = 1.0f;
	printf("Mag: loaded default calibration data\n");
	return ESP_OK;
}

esp_err_t mpu9250_mag_load_calibration_params(
		mpu9250_handle_t mpu9250_handle) {
	nvs_handle_t nvs_handle = (nvs_handle_t)mpu9250_handle->data.nvs_cal_data;

    // Lettura del blob di 48 byte dalla memoria flash
    uint8_t blob[48];
    size_t blob_size = sizeof(blob);
    esp_err_t err = nvs_get_blob(nvs_handle, "mag_model", blob, &blob_size);
    if (err != ESP_OK) {
    	ESP_LOGW(TAG, "Mag Calibration Params are not flashed: %s\n", esp_err_to_name(err));
        ESP_ERROR_CHECK(mpu9250_mag_load_default_calibration_params(mpu9250_handle));
        return ESP_OK;
    }

    // Controllo della dimensione del blob
    if (blob_size != 48) {
    	ESP_LOGW(TAG, "Mag params: Dimensione del blob non corretta (%d). I load default params ...", blob_size);
        ESP_ERROR_CHECK(mpu9250_mag_load_default_calibration_params(mpu9250_handle));
        return ESP_OK;

    }

    // Conversione del blob in una matrice float 3x3
    memcpy(mpu9250_handle->data.mag.cal.factors, blob, 36);
    memcpy(mpu9250_handle->data.mag.cal.offsets, (blob+36), 12);
    ESP_LOGI(TAG, "Mag matrix:\n[[%5.5f,%5.5f,%5.5f]\n,[%5.5f,%5.5f,%5.5f]\n,[%5.5f,%5.5f,%5.5f]]\n",
    		mpu9250_handle->data.mag.cal.factors[X_POS][X_POS],
    		mpu9250_handle->data.mag.cal.factors[X_POS][Y_POS],
    		mpu9250_handle->data.mag.cal.factors[X_POS][Z_POS],
    		mpu9250_handle->data.mag.cal.factors[Y_POS][X_POS],
    		mpu9250_handle->data.mag.cal.factors[Y_POS][Y_POS],
    		mpu9250_handle->data.mag.cal.factors[Y_POS][Z_POS],
    		mpu9250_handle->data.mag.cal.factors[Z_POS][X_POS],
    		mpu9250_handle->data.mag.cal.factors[Z_POS][Y_POS],
    		mpu9250_handle->data.mag.cal.factors[Z_POS][Z_POS]
    		);
    ESP_LOGI(TAG, "Mag bias:\n [%5.5f,%5.5f,%5.5f]\n",
	   		mpu9250_handle->data.mag.cal.offsets[X_POS],
	   		mpu9250_handle->data.mag.cal.offsets[Y_POS],
	   		mpu9250_handle->data.mag.cal.offsets[Z_POS]
    		);
	return ESP_OK;
}

esp_err_t mpu9250_mag_save_calibration_params(mpu9250_handle_t mpu9250_handle, char* data, int data_len) {
    if (data_len == 48) {  // 3x3 matrix + 3x1 bias = 12 floats = 12 * sizeof(float) = 12x4 = 48 bytes
        // Save the matrix to flash
    	ESP_ERROR_CHECK(nvs_set_blob((nvs_handle_t)mpu9250_handle->data.nvs_cal_data, "mag_model", data, data_len));
        nvs_commit((nvs_handle_t)mpu9250_handle->data.nvs_cal_data);
        ESP_LOGI(TAG, "Mag matrix+bias saved to flash");
    } else {
    	ESP_LOGE(TAG, "Mag Invalid matrix+bias data length");
    }
	return ESP_OK;
}

static esp_err_t mpu9250_mag_prepare(mpu9250_handle_t mpu9250_handle) {
	/******************************************************************************************
	 * AK8963
	 ******************************************************************************************/
	// Read AK8963 who am i
	uint8_t ak8963_whoAmI = 0xFF;
	ESP_ERROR_CHECK(
			ReadAk8963Register(mpu9250_handle, AK8963_WIA));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(
			mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &ak8963_whoAmI));

	if (ak8963_whoAmI != AK8963_WHO_AM_I_RESULT) {
		printf("MPU9250: AK8963 CONFIGURATION FAILED: [%d]\n", ak8963_whoAmI);
		return ESP_FAIL;
	} else {
		printf("MPU9250: (AK8963 communication OK)\n");
	}

    // Read calibration data in fuse mode
    ESP_ERROR_CHECK(
    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, AK8963_FUSE_ROM));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(ReadAk8963Registers(mpu9250_handle, AK8963_ASAX, 3));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(mpu9250_read_buff(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, mpu9250_handle->data.mag.factory_cal.asa.array, 3*8));

    uint8_t asax = mpu9250_handle->data.mag.factory_cal.asa.array[X_POS];
    uint8_t asay = mpu9250_handle->data.mag.factory_cal.asa.array[Y_POS];
    uint8_t asaz = mpu9250_handle->data.mag.factory_cal.asa.array[Z_POS];
    mpu9250_handle->data.mag.factory_cal.asa.array[X_POS] =  asax;
    mpu9250_handle->data.mag.factory_cal.asa.array[Y_POS] =  asay;
    mpu9250_handle->data.mag.factory_cal.asa.array[Z_POS] =  asaz;

    for(uint8_t i = 0; i < 3; i++) {
    	mpu9250_handle->data.mag.factory_cal.factors.array[i] = (((float)(mpu9250_handle->data.mag.factory_cal.asa.array[i] - 128))/2.0f / 128.0f + 1.0f) * (4912.0f / 32768.0f);
    }
    printf("mag_asa [%d, %d, %d]\n", mpu9250_handle->data.mag.factory_cal.asa.array[X_POS], mpu9250_handle->data.mag.factory_cal.asa.array[Y_POS], mpu9250_handle->data.mag.factory_cal.asa.array[Z_POS]);
    printf("mag_scale [%5.5f, %5.5f, %5.5f]\n", mpu9250_handle->data.mag.factory_cal.factors.array[X_POS], mpu9250_handle->data.mag.factory_cal.factors.array[Y_POS], mpu9250_handle->data.mag.factory_cal.factors.array[Z_POS]);

    ESP_ERROR_CHECK(
    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, AK8963_PWR_DOWN));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read AK8963 who am i
    ak8963_whoAmI = 0xFF;
    ESP_ERROR_CHECK(ReadAk8963Register(mpu9250_handle, AK8963_WIA));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(
			mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &ak8963_whoAmI));

    if(ak8963_whoAmI != AK8963_WHO_AM_I_RESULT) {
    	printf("MPU9250: AK8963 CONFIGURATION FAILED: [%d]\n", ak8963_whoAmI);
    	return ESP_FAIL;
    } else {
    	printf("MPU9250: (AK8963 communication OK)\n");
    }
    ESP_ERROR_CHECK(mpu9250_mag_set_mode2_with_precision(mpu9250_handle, INV_MAG_PRECISION_16_BITS));
	printf("MPU9250: (AK8963 configuration OK)\n");

	return ESP_OK;

}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t mpu9250_mag_init(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_mag_prepare(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_mag_load_calibration_params(mpu9250_handle));

	// set calibrated data to zero ..
	mpu9250_handle->data.cal_data.data_s_xyz.mag_data_x = 0.0f;
	mpu9250_handle->data.cal_data.data_s_xyz.mag_data_y = 0.0f;
	mpu9250_handle->data.cal_data.data_s_xyz.mag_data_z = 0.0f;

	printf("Mag offsets: [%3.5f,%3.5f,%3.5f]\n", mpu9250_handle->data.mag.cal.offsets[X_POS], mpu9250_handle->data.mag.cal.offsets[Y_POS], mpu9250_handle->data.mag.cal.offsets[Z_POS]);
	printf("Mag factors2: [%2.5f,%2.5f,%2.5f]\n", mpu9250_handle->data.mag.cal.factors[X_POS][X_POS], mpu9250_handle->data.mag.cal.factors[Y_POS][Y_POS], mpu9250_handle->data.mag.cal.factors[Z_POS][Z_POS]);
	ESP_ERROR_CHECK(mpu9250_mag_set_continuous_reading(mpu9250_handle));
	return ESP_OK;
}
esp_err_t mpu9250_mag_load_cal_data(mpu9250_handle_t mpu9250_handle) {
	if(mpu9250_handle->data.mag.drdy) {
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_x = mpu9250_handle->data.mag.cal.factors[X_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_x - mpu9250_handle->data.mag.cal.offsets[X_POS]) + mpu9250_handle->data.mag.cal.factors[X_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_y - mpu9250_handle->data.mag.cal.offsets[Y_POS]) + mpu9250_handle->data.mag.cal.factors[X_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_z - mpu9250_handle->data.mag.cal.offsets[Z_POS]);
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_y = mpu9250_handle->data.mag.cal.factors[Y_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_x - mpu9250_handle->data.mag.cal.offsets[X_POS]) + mpu9250_handle->data.mag.cal.factors[Y_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_y - mpu9250_handle->data.mag.cal.offsets[Y_POS]) + mpu9250_handle->data.mag.cal.factors[Y_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_z - mpu9250_handle->data.mag.cal.offsets[Z_POS]);
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_z = mpu9250_handle->data.mag.cal.factors[Z_POS][X_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_x - mpu9250_handle->data.mag.cal.offsets[X_POS]) + mpu9250_handle->data.mag.cal.factors[Z_POS][Y_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_y - mpu9250_handle->data.mag.cal.offsets[Y_POS]) + mpu9250_handle->data.mag.cal.factors[Z_POS][Z_POS]*(mpu9250_handle->data.raw_data.data_s_xyz.mag_data_z - mpu9250_handle->data.mag.cal.offsets[Z_POS]);
	} else {
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_x = 0.0f;
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_y = 0.0f;
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_z = 0.0f;
	}
	return ESP_OK;
}
esp_err_t mpu9250_mag_load_precision(mpu9250_handle_t mpu9250_handle) {
	uint8_t mag_conf = 0;

	// read cntl1
    ESP_ERROR_CHECK(ReadAk8963Register(mpu9250_handle, AK8963_CNTL1));
    vTaskDelay(pdMS_TO_TICKS(10));
	ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, &mag_conf));

	mpu9250_handle->data.mag.precision = ((AK8963_PRECISION_MASK & mag_conf) ? INV_MAG_PRECISION_16_BITS : INV_MAG_PRECISION_14_BITS);

	switch(mpu9250_handle->data.mag.precision) {
	case INV_MAG_PRECISION_14_BITS: {
		printf("MPU9250: Mag Precision 0.6 \n");
		break;
	}
	case INV_MAG_PRECISION_16_BITS: {
		printf("MPU9250: Mag Precision 0.15 \n");
		break;
	}
	default: {
		printf("MPU9250: Mag Precision UNKNOWN [%d]\n", mpu9250_handle->data.mag.precision);
		break;
	}
	}
	return ESP_OK;
}

esp_err_t mpu9250_mag_set_precision(mpu9250_handle_t mpu9250_handle, uint8_t precision) {
	mpu9250_handle->data.mag.precision=precision;
	ESP_ERROR_CHECK(mpu9250_mag_save_precision(mpu9250_handle));
	return ESP_OK;
}

esp_err_t mpu9250_mag_set_mode2_with_precision(mpu9250_handle_t mpu9250_handle, uint8_t precision) {
	// Configure AK8963 Continuous Mode
	switch(precision) {
	case INV_MAG_PRECISION_14_BITS: {
	    ESP_ERROR_CHECK(
	    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, 0x06));  // mode2 (100Hz) 14 bits
		break;
	}
	case INV_MAG_PRECISION_16_BITS: {
	    ESP_ERROR_CHECK(
	    		WriteAk8963Register(mpu9250_handle, AK8963_CNTL1, 0x16));  // mode2 (100Hz) 16bits
		break;
	}
	default: {
		return ESP_FAIL;
	}
	}
    mpu9250_handle->data.mag.precision = precision;
    ESP_ERROR_CHECK(mpu9250_mag_calc_precision_factor(mpu9250_handle));

    return ESP_OK;
}


esp_err_t mpu9250_mag_set_continuous_reading(mpu9250_handle_t mpu9250_handle) {
    // Mpu9250 start reading AK8963
    vTaskDelay(pdMS_TO_TICKS(100));
    ReadAk8963Registers(mpu9250_handle, AK8963_ST1, 8);
	printf("MPU9250: continuous reading for AK8963 on SLV0\n");
	return ESP_OK;
}

