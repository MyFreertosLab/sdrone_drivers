/*
 * mcpwm.h
 *
 *  Created on: 23 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MOTORS_INCLUDE_MOTORS_H_
#define COMPONENTS_MOTORS_INCLUDE_MOTORS_H_
#include <stdint.h>
#include <esp_err.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>

#define MOTORS_MAX_NUM 12
#define MOTORS_FREQUENCY 16 // 16KHz
#define MOTORS_PWM_FREQUENCY 490 // 490 Hz
#define MOTORS_IGNORE_PIN (-1)
#define MOTORS_SWITCH_ON_OFF_PIN GPIO_NUM_32
#define MOTORS_SWITCH_ON_OFF_PIN_SEL GPIO_SEL_32
#define MOTORS_SWITCH_ON  0
#define MOTORS_SWITCH_OFF 1

//#define MOTORS_FRAME_HORIZONTAL_HEXACOPTER
#define MOTORS_FRAME_TWO_HORIZONTAL_AXIS
#define MOTORS_DUTY_MAX_ZERO  52
#define MOTORS_DUTY_MAX_LOW   66
#define MOTORS_DUTY_MAX      100
//#define MOTORS_DUTY_TO_ACCEL_FACTOR_LOW 0.0602f
#define MOTORS_DUTY_TO_NEWTON_FACTOR_LOW 0.105f
#define MOTORS_DUTY_TO_NEWTON_FACTOR_HIGH 0.105f
#define MOTORS_ACCEL_RANGE 4.4f
#define MOTORS_DUTY_DEAD_RANGE 60.0f

// TODO: Define all positions from quad to opto
typedef enum {
	FRONT,
	REAR,
	LEFT,
	RIGHT,
	FRONT_LEFT,
	FRONT_RIGHT,
	REAR_LEFT,
	REAR_RIGHT,
	FRONT_TOP_LEFT,
	FRONT_TOP_RIGHT,
	REAR_TOP_LEFT,
	REAR_TOP_RIGHT,
	FRONT_BOTTOM_LEFT,
	FRONT_BOTTOM_RIGHT,
	REAR_BOTTOM_LEFT,
	REAR_BOTTOM_RIGHT
} motor_position_t;

typedef enum {
	ON_DISARMED,
	ON_ARMED,
	OFF
} motors_status_t;

typedef struct {
	bool enabled;
	uint8_t num;
	int pin;
	motor_position_t position;
	mcpwm_unit_t mcpwm;
	float duty_cycle;
} motor_t;

typedef struct {
	motor_t motor[MOTORS_MAX_NUM];
	uint32_t frequency;
	uint8_t motors_num;
	int switch_on_off_pin;
	motors_status_t status;
} motors_t;

typedef motors_t* motors_handle_t;

esp_err_t motors_init(motors_handle_t motors_handle);
esp_err_t motors_arm(motors_handle_t motors_handle);
esp_err_t motors_disarm(motors_handle_t motors_handle);
esp_err_t motors_switchoff(motors_handle_t motors_handle);
esp_err_t motors_switchon(motors_handle_t motors_handle);
esp_err_t motors_update(motors_handle_t motors_handle);
esp_err_t motors_newton_to_duty(float newton, float* duty);
esp_err_t motors_duty_to_newton(float duty, float* newton);

#endif /* COMPONENTS_MOTORS_INCLUDE_MOTORS_H_ */
