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
//#define MOTORS_FRAME_TWO_HORIZONTAL_AXIS
#define MOTORS_FRAME_X_QUADCOPTER

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
#define NUM_MOTORS 6
#else
#ifdef MOTORS_FRAME_X_QUADCOPTER
#define NUM_MOTORS 4
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
#define NUM_MOTORS 2
#endif
#endif
#endif

#define MOTORS_THRUST_MAX  4.4f
#define MOTORS_THRUST_MIN  0.0f
#define MOTORS_THRUST_RANGE (MOTORS_THRUST_MAX - MOTORS_THRUST_MIN) // m/s^2 for each motor

#define MOTORS_DUTY_MIN (0.001f*(float)MOTORS_PWM_FREQUENCY*100.0f)
#define MOTORS_DUTY_MAX (0.002f*(float)MOTORS_PWM_FREQUENCY*100.0f)
#define MOTORS_DUTY_RANGE (MOTORS_DUTY_MAX - MOTORS_DUTY_MIN)

#define MOTORS_THRUST_TO_DUTY_FACTOR (MOTORS_DUTY_RANGE/MOTORS_THRUST_RANGE)
#define MOTORS_DUTY_TO_THRUST_FACTOR (MOTORS_THRUST_RANGE/MOTORS_DUTY_RANGE)
#define MOTORS_THRUST_TO_DUTY(thrust) ((thrust-MOTORS_THRUST_MIN)*MOTORS_THRUST_TO_DUTY_FACTOR+MOTORS_DUTY_MIN)
#define MOTORS_DUTY_TO_THRUST(duty) ((duty-MOTORS_DUTY_MIN)*MOTORS_DUTY_TO_THRUST_FACTOR+MOTORS_THRUST_MIN)

//#define MOTORS_ENABLE_SWITCHON_SWITCHOFF

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
	MOTORS_DISARMED,
	MOTORS_ARMED,
	MOTORS_OFF
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
	float at[3];
	float thrust;
	float thrust_ms;
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
esp_err_t motors_thrust_to_duty(float newton, float* duty);
esp_err_t motors_duty_to_thrust(float duty, float* newton);

#endif /* COMPONENTS_MOTORS_INCLUDE_MOTORS_H_ */
