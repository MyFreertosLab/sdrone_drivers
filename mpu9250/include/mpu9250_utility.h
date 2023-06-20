/*
 * mpu9250_utility.h
 *
 *  Created on: 20 giu 2023
 *      Author: andrea
 */

#ifndef COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_UTILITY_H_
#define COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_UTILITY_H_
#include <esp_err.h>

/*********************************
*********** Utilities ************
*********************************/
typedef union {
   int8_t array[3];
   struct {
     int8_t x;
     int8_t y;
     int8_t z;
   } xyz;
} mpu9250_int8_3d_t;

typedef union {
   uint8_t array[3];
   struct {
     uint8_t x;
     uint8_t y;
     uint8_t z;
   } xyz;
} mpu9250_uint8_3d_t;

typedef union {
   int16_t array[3];
   struct {
     int16_t x;
     int16_t y;
     int16_t z;
   } xyz;
} mpu9250_int_3d_t;

typedef union {
   int32_t array[3];
   struct {
     int32_t x;
     int32_t y;
     int32_t z;
   } xyz;
} mpu9250_int32_3d_t;

typedef union {
   int64_t array[3];
   struct {
     int64_t x;
     int64_t y;
     int64_t z;
   } xyz;
} mpu9250_int64_3d_t;

typedef union {
   uint16_t array[3];
   struct {
     uint16_t x;
     uint16_t y;
     uint16_t z;
   } xyz;
} mpu9250_uint_3d_t;

typedef union {
   uint32_t array[3];
   struct {
     uint32_t x;
     uint32_t y;
     uint32_t z;
   } xyz;
} mpu9250_uint32_3d_t;

typedef union {
   uint64_t array[3];
   struct {
     uint64_t x;
     uint64_t y;
     uint64_t z;
   } xyz;
} mpu9250_uint64_3d_t;

typedef union {
   double array[3];
   struct {
     double x;
     double y;
     double z;
   } xyz;
} mpu9250_double_3d_t;

typedef union {
   float array[3];
   struct {
     float x;
     float y;
     float z;
   } xyz;
} mpu9250_float_3d_t;


#endif /* COMPONENTS_DRIVERS_MPU9250_INCLUDE_MPU9250_UTILITY_H_ */
