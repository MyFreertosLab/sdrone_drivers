/*
 * mpu9250_gyroel.h
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MPU9250_INCLUDE_MPU9250_GYRO_H_
#define COMPONENTS_MPU9250_INCLUDE_MPU9250_GYRO_H_

#include <mpu9250.h>
esp_err_t mpu9250_gyro_init(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_gyro_load_fsr(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_gyro_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr);
esp_err_t mpu9250_gyro_load_device_offset(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_gyro_set_device_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff);
esp_err_t mpu9250_gyro_load_cal_data(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_gyro_save_calibration_params(mpu9250_handle_t mpu9250_handle, char* data, int data_len);
esp_err_t mpu9250_gyro_load_calibration_params(mpu9250_handle_t mpu9250_handle);

#endif /* COMPONENTS_MPU9250_INCLUDE_MPU9250_GYRO_H_ */

