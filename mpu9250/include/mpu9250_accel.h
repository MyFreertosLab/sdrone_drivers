/*
 * mpu9250_accel.h
 *
 *  Created on: 6 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MPU9250_INCLUDE_MPU9250_ACCEL_H_
#define COMPONENTS_MPU9250_INCLUDE_MPU9250_ACCEL_H_

#include <mpu9250.h>
esp_err_t mpu9250_acc_init(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_acc_load_fsr(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_acc_set_fsr(mpu9250_handle_t mpu9250_handle, uint8_t fsr);
esp_err_t mpu9250_acc_load_offset(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_acc_set_offset(mpu9250_handle_t mpu9250_handle, int16_t xoff, int16_t yoff, int16_t zoff);
esp_err_t mpu9250_acc_update_state(mpu9250_handle_t mpu9250_handle);

#endif /* COMPONENTS_MPU9250_INCLUDE_MPU9250_ACCEL_H_ */
