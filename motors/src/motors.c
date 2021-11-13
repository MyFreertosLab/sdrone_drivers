#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <motors.h>

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
static mcpwm_generator_t motors_pwm_generator[MOTORS_MAX_NUM] = {
		MCPWM_GEN_A, MCPWM_GEN_B, MCPWM_GEN_A, MCPWM_GEN_B, MCPWM_GEN_A, MCPWM_GEN_B,
		MCPWM_GEN_A, MCPWM_GEN_B, MCPWM_GEN_A, MCPWM_GEN_B, MCPWM_GEN_A, MCPWM_GEN_B
};

static mcpwm_timer_t motors_timers[MOTORS_MAX_NUM] = {
		MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_1, MCPWM_TIMER_2, MCPWM_TIMER_2,
		MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_1, MCPWM_TIMER_2, MCPWM_TIMER_2
};

float motors_get_duty_low(motors_handle_t motors_handle) {
    return motors_handle->frequency/10.;
}
float motors_get_duty_high(motors_handle_t motors_handle) {
    return 2.0*motors_get_duty_low(motors_handle);
}
uint8_t motors_get_mcpwm_motors_range(mcpwm_unit_t unit) {
	return (unit == MCPWM_UNIT_1 ? 6 : 0);
}

esp_err_t motors_init_pwm_timer(motors_handle_t motors_handle, mcpwm_unit_t unit, mcpwm_io_signals_t timer) {
	printf("motors: motors_init_pwm_timer [%d/%d]\n", unit, timer);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = motors_handle->frequency;

    float duty = motors_get_duty_low(motors_handle);
    pwm_config.cmpr_a = duty;
    pwm_config.cmpr_b = duty;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    ESP_ERROR_CHECK(mcpwm_init(unit, timer, &pwm_config));
    return ESP_OK;
}

esp_err_t motors_low_duty_motor(motors_handle_t motors_handle, uint8_t motor_indx) {
	ESP_ERROR_CHECK(mcpwm_set_duty(motors_handle->motor[motor_indx].mcpwm, motors_timers[motor_indx],motors_pwm_generator[motor_indx], motors_get_duty_low(motors_handle)));
	return ESP_OK;
}

esp_err_t motors_high_duty_motor(motors_handle_t motors_handle, uint8_t motor_indx) {
	ESP_ERROR_CHECK(mcpwm_set_duty(motors_handle->motor[motor_indx].mcpwm, motors_timers[motor_indx],motors_pwm_generator[motor_indx],motors_get_duty_high(motors_handle)));
	return ESP_OK;
}

esp_err_t motors_low_duty_motors(motors_handle_t motors_handle) {
	for (uint8_t i = 0; i < MOTORS_MAX_NUM; i++) {
		if (motors_handle->motor[i].enabled) {
			ESP_ERROR_CHECK(motors_low_duty_motor(motors_handle, i));
		}
	}
	return ESP_OK;
}

esp_err_t motors_high_duty_motors(motors_handle_t motors_handle) {
	for (uint8_t i = 0; i < MOTORS_MAX_NUM; i++) {
		if (motors_handle->motor[i].enabled) {
			ESP_ERROR_CHECK(motors_high_duty_motor(motors_handle, i));
		}
	}
	return ESP_OK;
}

esp_err_t motors_update_mcpwm_duty(motors_handle_t motors_handle) {
	for (uint8_t i = 0; i < MOTORS_MAX_NUM; i++) {
	  if (motors_handle->motor[i].enabled) {
		ESP_ERROR_CHECK(mcpwm_set_duty(motors_handle->motor[i].mcpwm, motors_timers[i],motors_pwm_generator[i],motors_handle->motor[i].duty_cycle));
      }
	}
	return ESP_OK;
}

esp_err_t motors_init_mcpwm_pins(motors_handle_t motors_handle, mcpwm_unit_t unit) {
    uint8_t si = motors_get_mcpwm_motors_range(unit);
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = motors_handle->motor[0+si].pin,
        .mcpwm0b_out_num = motors_handle->motor[1+si].pin,
        .mcpwm1a_out_num = motors_handle->motor[2+si].pin,
        .mcpwm1b_out_num = motors_handle->motor[3+si].pin,
        .mcpwm2a_out_num = motors_handle->motor[4+si].pin,
        .mcpwm2b_out_num = motors_handle->motor[5+si].pin,
		// TODO: Vedere se necessari
        .mcpwm_sync0_in_num  = -1,
        .mcpwm_sync1_in_num  = -1,
        .mcpwm_sync2_in_num  = -1,
        .mcpwm_fault0_in_num = -1,
        .mcpwm_fault1_in_num = -1,
        .mcpwm_fault2_in_num = -1,
        .mcpwm_cap0_in_num   = -1,
        .mcpwm_cap1_in_num   = -1,
        .mcpwm_cap2_in_num   = -1
    };
    gpio_pullup_en(motors_handle->motor[0+si].pin);
    gpio_pullup_en(motors_handle->motor[1+si].pin);
    gpio_pullup_en(motors_handle->motor[2+si].pin);
    gpio_pullup_en(motors_handle->motor[3+si].pin);
    gpio_pullup_en(motors_handle->motor[4+si].pin);
    gpio_pullup_en(motors_handle->motor[5+si].pin);

    ESP_ERROR_CHECK(mcpwm_set_pin(unit, &pin_config));
    return ESP_OK;
}
esp_err_t motors_init_mcpwm(motors_handle_t motors_handle, mcpwm_unit_t unit) {
	printf("motors: motors_init_mcpwm [%d]\n", unit);
	ESP_ERROR_CHECK(motors_init_mcpwm_pins(motors_handle, unit));
    uint8_t si = motors_get_mcpwm_motors_range(unit);

    ESP_ERROR_CHECK(motors_init_pwm_timer(motors_handle, unit, MCPWM_TIMER_0));
    ESP_ERROR_CHECK(motors_init_pwm_timer(motors_handle, unit, MCPWM_TIMER_1));
    ESP_ERROR_CHECK(motors_init_pwm_timer(motors_handle, unit, MCPWM_TIMER_2));

    // Sync timer 1 and timer 2 with timer 0
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, 1, 0);
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, 1, 0);
    MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].sync.out_sel = 1;

    for(uint8_t i = 0; i < 6; i++) {
      if(motors_handle->motor[i+si].enabled) {
        motors_handle->motor[i+si].mcpwm = unit;
        motors_handle->motor[i+si].duty_cycle = mcpwm_get_duty(unit, motors_timers[i+si], motors_pwm_generator[i+si]);
      }
    }
    printf("motors::motors_init_mcpwm: END\n");
    return ESP_OK;
};

/*
 * Hexacopter with horizontal axis
 *
 *        3   5
 *         \ /
 *      2---+---1
 *         / \
 *        6   4
 */
#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
esp_err_t motors_config(motors_handle_t motors_handle) {
	printf("motors_config_horizontal_hexacopter::Started\n");

	// Config switchon/off pin and frequency
	motors_handle->frequency = MOTORS_PWM_FREQUENCY;

	// Config motor left
	motors_handle->motor[0].enabled = true;
	motors_handle->motor[0].pin = GPIO_NUM_21;
	motors_handle->motor[0].position = LEFT;
    motors_handle->motor[0].num = 2;

	// Config motor right
	motors_handle->motor[1].enabled = true;
	motors_handle->motor[1].pin = GPIO_NUM_22;
	motors_handle->motor[1].position = RIGHT;
    motors_handle->motor[1].num = 1;

	// Config motor front left
	motors_handle->motor[2].enabled = true;
	motors_handle->motor[2].pin = GPIO_NUM_18;
	motors_handle->motor[2].position = FRONT_LEFT;
    motors_handle->motor[2].num = 3;

	// Config motor front right
	motors_handle->motor[3].enabled = true;
	motors_handle->motor[3].pin = GPIO_NUM_19;
	motors_handle->motor[3].position = FRONT_RIGHT;
    motors_handle->motor[3].num = 5;

	// Config motor rear left
	motors_handle->motor[4].enabled = true;
	motors_handle->motor[4].pin = GPIO_NUM_5;
	motors_handle->motor[4].position = REAR_LEFT;
    motors_handle->motor[4].num = 6;

	// Config motor rear right
	motors_handle->motor[5].enabled = true;
	motors_handle->motor[5].pin = GPIO_NUM_4;
	motors_handle->motor[5].position = REAR_RIGHT;
    motors_handle->motor[5].num = 4;


	printf("motors_config_horizontal_hexacopter::Ended\n");

	return ESP_OK;
}
// convert tangential acceleration on axis to duty cycle on motors
esp_err_t motors_axis_at_to_motors_duty(motors_handle_t motors_handle) {
	// TODO: T.B.D.
	return ESP_OK;
}
#else
/*
 * Quadcopter with horizontal axis
 *
 *        3   1
 *         \ /
 *          +
 *         / \
 *        2   4
 */
#ifdef MOTORS_FRAME_X_QUADCOPTER
esp_err_t motors_config(motors_handle_t motors_handle) {
       printf("motors_config_x_quadcopter::Started\n");

       // Config switchon/off pin and frequency
       motors_handle->frequency = MOTORS_PWM_FREQUENCY;

       // Config motor front right
       motors_handle->motor[0].enabled = true;
       motors_handle->motor[0].pin = GPIO_NUM_22; // j14 on board
       motors_handle->motor[0].position = FRONT_RIGHT;
       motors_handle->motor[0].num = 1;

       // Config motor rear left
       motors_handle->motor[1].enabled = true;
       motors_handle->motor[1].pin = GPIO_NUM_18; // j8 on board
       motors_handle->motor[1].position = REAR_LEFT;
       motors_handle->motor[1].num = 2;

       // Config motor front left
       motors_handle->motor[2].enabled = true;
       motors_handle->motor[2].pin = GPIO_NUM_4; // j13 on board // provvisorio .. da rimettere su GPIO_NUM_19 (j7 on board)
       motors_handle->motor[2].position = FRONT_LEFT;
       motors_handle->motor[2].num = 3;

       // Config motor rear right
       motors_handle->motor[3].enabled = true;
       motors_handle->motor[3].pin = GPIO_NUM_21; // j12 on board
       motors_handle->motor[3].position = REAR_RIGHT;
       motors_handle->motor[3].num = 4;


       printf("motors_config_x_quadcopter::Ended\n");

       return ESP_OK;
}

// convert tangential acceleration on axis to duty cycle on motors
esp_err_t motors_axis_at_to_motors_duty(motors_handle_t motors_handle) {

	float at[4] = {0.0f,0.0f,0.0f,0.0f};
	// motors 1 & 2 are counterclockwise
	// motors 3 & 4 are clockwise
	if(motors_handle->thrust >= 0.20f) {
		at[0] = (- motors_handle->at[0] - motors_handle->at[1] - motors_handle->at[2])*0.25f + motors_handle->thrust;
		at[1] = (  motors_handle->at[0] + motors_handle->at[1] - motors_handle->at[2])*0.25f + motors_handle->thrust;
		at[2] = (  motors_handle->at[0] - motors_handle->at[1] + motors_handle->at[2])*0.25f + motors_handle->thrust;
		at[3] = (- motors_handle->at[0] + motors_handle->at[1] + motors_handle->at[2])*0.25f + motors_handle->thrust;
	}

	for(uint8_t i = 0; i < 4; i++) {
		ESP_ERROR_CHECK(motors_newton_to_duty(at[i], &motors_handle->motor[i].duty_cycle));
	}

	return ESP_OK;
}
#else
/*
 * Two Horizontal Axis Test Frame (pitch rotation used to test motors controller algorithm)
 *
 *      2---+---1
 *         / \
 *        -----
 */
#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
esp_err_t motors_config(motors_handle_t motors_handle) {
	printf("motors_config_two_horizontal_axis::Started\n");

	// Config switchon/off pin and frequency
	motors_handle->frequency = MOTORS_PWM_FREQUENCY;

	// Config motor left
	motors_handle->motor[0].enabled = true;
	motors_handle->motor[0].pin = GPIO_NUM_18;
	motors_handle->motor[0].position = LEFT;
    motors_handle->motor[0].num = 2;

	// Config motor right
	motors_handle->motor[1].enabled = true;
	motors_handle->motor[1].pin = GPIO_NUM_19;
	motors_handle->motor[1].position = RIGHT;
    motors_handle->motor[1].num = 1;

	printf("motors_config_two_horizontal_axis::Ended\n");

	return ESP_OK;
}
// convert tangential acceleration on axis to duty cycle on motors
esp_err_t motors_axis_at_to_motors_duty(motors_handle_t motors_handle) {
	// TODO: T.B.D.
	return ESP_OK;
}
#endif
#endif
#endif
esp_err_t motors_config_switchonoff_pin(motors_handle_t motors_handle) {
	printf("motors_config_switchonoff_pin::Started\n");
    // Gpio Switch On/Off pin
	motors_handle->switch_on_off_pin = MOTORS_SWITCH_ON_OFF_PIN;
    gpio_pulldown_en(MOTORS_SWITCH_ON_OFF_PIN);
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = MOTORS_SWITCH_ON_OFF_PIN_SEL;
    gpio_config(&gp);
	printf("motors_config_switchonoff_pin::Ended\n");
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t motors_newton_to_duty(float newton, float* duty) {
	float _newton = newton;
	float _duty = *duty;

	// limit newton in [0,MOTORS_ACCEL_RANGE]
	if(_newton < 0.0f) {
		_newton = 0.0f;
	} else if(_newton > MOTORS_ACCEL_RANGE) {
		_newton = MOTORS_ACCEL_RANGE;
	}

	// change duty in soft mode (80%old and 20% new at 490Hz)
	// without this trick can breakout the motors
	if(_newton <= (MOTORS_DUTY_DEAD_RANGE - MOTORS_DUTY_MAX_ZERO)*MOTORS_DUTY_TO_NEWTON_FACTOR_LOW) {
		*duty = 0.8f*_duty + 0.2f*MOTORS_DUTY_MAX_ZERO;
	} else if(_newton <= (MOTORS_DUTY_MAX_LOW - MOTORS_DUTY_MAX_ZERO)*MOTORS_DUTY_TO_NEWTON_FACTOR_LOW) {
		*duty = 0.8f*_duty + 0.2f*(MOTORS_DUTY_MAX_ZERO + _newton/MOTORS_DUTY_TO_NEWTON_FACTOR_LOW);
	} else {
		*duty = 0.8f*_duty + 0.2f*(MOTORS_DUTY_MAX_LOW + (_newton - (MOTORS_DUTY_MAX_LOW - MOTORS_DUTY_MAX_ZERO)*MOTORS_DUTY_TO_NEWTON_FACTOR_LOW)/MOTORS_DUTY_TO_NEWTON_FACTOR_HIGH);
	}
	return ESP_OK;
}

esp_err_t motors_duty_to_newton(float duty, float* newton) {
	if(duty <= MOTORS_DUTY_DEAD_RANGE) {
		*newton = 0.0f;
	} else if(duty < MOTORS_DUTY_MAX_LOW) {
		*newton = (duty - MOTORS_DUTY_MAX_ZERO)*MOTORS_DUTY_TO_NEWTON_FACTOR_LOW;
	} else {
		*newton = (MOTORS_DUTY_MAX_LOW-MOTORS_DUTY_MAX_ZERO)*MOTORS_DUTY_TO_NEWTON_FACTOR_LOW + (duty - MOTORS_DUTY_MAX_LOW)*MOTORS_DUTY_TO_NEWTON_FACTOR_HIGH;
	}
	return ESP_OK;
}

esp_err_t motors_init(motors_handle_t motors_handle) {
	printf("motors: motors_init\n");
	memset(motors_handle, 0, sizeof(*motors_handle));
#ifdef MOTORS_ENABLE_SWITCHON_SWITCHOFF
	ESP_ERROR_CHECK(motors_config_switchonoff_pin(motors_handle));
	ESP_ERROR_CHECK(motors_switchoff(motors_handle));
    vTaskDelay(pdMS_TO_TICKS(5000)); //delay of 5s
#endif
	ESP_ERROR_CHECK(motors_config(motors_handle));

	ESP_ERROR_CHECK(motors_init_mcpwm(motors_handle, MCPWM_UNIT_0));
	ESP_ERROR_CHECK(motors_init_mcpwm(motors_handle, MCPWM_UNIT_1));
    vTaskDelay(500); //delay of 5s (at 100Hz)

#ifdef MOTORS_ENABLE_SWITCHON_SWITCHOFF
    ESP_ERROR_CHECK(motors_high_duty_motors(motors_handle));
//	ESP_ERROR_CHECK(motors_switchon(motors_handle));
    vTaskDelay(pdMS_TO_TICKS(5000)); //delay of 2s
#else
	ESP_ERROR_CHECK(motors_disarm(motors_handle));
#endif
    ESP_ERROR_CHECK(motors_low_duty_motors(motors_handle));
    vTaskDelay(pdMS_TO_TICKS(2000)); //delay of 2s
	return ESP_OK;
}

esp_err_t motors_arm(motors_handle_t motors_handle) {
    ESP_ERROR_CHECK(motors_low_duty_motors(motors_handle));
    motors_handle->status = MOTORS_ARMED;
	return ESP_OK;
}

esp_err_t motors_disarm(motors_handle_t motors_handle) {
    ESP_ERROR_CHECK(motors_low_duty_motors(motors_handle));;
    motors_handle->status = MOTORS_DISARMED;
	return ESP_OK;
}
esp_err_t motors_switchoff(motors_handle_t motors_handle) {
    gpio_set_level(MOTORS_SWITCH_ON_OFF_PIN, MOTORS_SWITCH_OFF);
    motors_handle->status = MOTORS_OFF;
	return ESP_OK;
}
esp_err_t motors_switchon(motors_handle_t motors_handle) {
    gpio_set_level(MOTORS_SWITCH_ON_OFF_PIN, MOTORS_SWITCH_ON);
	motors_handle->status = MOTORS_DISARMED;
	return ESP_OK;
}
esp_err_t motors_update(motors_handle_t motors_handle) {
	if(motors_handle->status == MOTORS_ARMED) {
	  ESP_ERROR_CHECK(motors_axis_at_to_motors_duty(motors_handle));
      ESP_ERROR_CHECK(motors_update_mcpwm_duty(motors_handle));
	}
	return ESP_OK;
}

