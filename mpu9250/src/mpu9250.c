/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers for mp9250
 *
 *  @{
 *      @file       mpu9250.c
 *      @brief      An SPI-based driver for mpu9250
 *      @details    This driver currently works for the following devices:
 *                  MPU6500
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <mpu9250_spi.h>
#include <mpu9250_accel.h>
#include <mpu9250_gyro.h>
#include <mpu9250_mag.h>
#include <driver/gpio.h>

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/

//This function is called with every interrupt
static void IRAM_ATTR mpu9250_isr(void* mpu9250_handle)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(((mpu9250_handle_t)mpu9250_handle)->data_ready_task_handle, &xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(); // this wakes up sample_timer_task immediately
	}
}

void mpu9250_cb_init(mpu9250_cb_handle_t cb) {
	cb->cursor = -1;
	memset(cb, 0, sizeof(mpu9250_cb_t));
}

void mpu9250_cb_add(mpu9250_cb_handle_t cb, int16_t val) {
	cb->cursor++;
	cb->cursor %= CIRCULAR_BUFFER_SIZE;
	cb->data[cb->cursor] = val;
}
void mpu9250_cb_means(mpu9250_cb_handle_t cb, int16_t* mean) {
	int64_t sum = 0;
	for(uint8_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++) {
		sum += cb->data[i];
	}
	*mean = sum/CIRCULAR_BUFFER_SIZE;
}
void mpu9250_cb_last(mpu9250_cb_handle_t cb, int16_t* val) {
	*val = cb->data[cb->cursor];
}


/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/

esp_err_t mpu9250_init(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_spi_init(mpu9250_handle));
	mpu9250_handle->data_ready_task_handle=xTaskGetCurrentTaskHandle();

	mpu9250_handle->data.speed_if[X_POS] = 0.0f;
	mpu9250_handle->data.speed_if[Y_POS] = 0.0f;
	mpu9250_handle->data.speed_if[Z_POS] = 0.0f;

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_cb_init(&mpu9250_handle->data.accel.cb[i]);
		mpu9250_cb_init(&mpu9250_handle->data.gyro.cb[i]);
		mpu9250_cb_init(&mpu9250_handle->data.gyro.cb_alfa[i]);
	}

	ESP_ERROR_CHECK(
			mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_1, CLKSEL_PLL));
	vTaskDelay(pdMS_TO_TICKS(10));

    // set Master Control
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_USER_CTRL, I2C_MST_EN));

    // I2C 400KHz
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_MST_CTRL, I2C_MST_CLK));
	vTaskDelay(pdMS_TO_TICKS(10));


	uint8_t i2c_mst_status = 0xFF;
    ESP_ERROR_CHECK(mpu9250_read8(mpu9250_handle, MPU9250_I2C_MST_STATUS, &i2c_mst_status));
    printf("I2C STATUS: [%d]\n", i2c_mst_status);


	/******************************************************************************************
	 * MAGNETOMETER
	 ******************************************************************************************/
	ESP_ERROR_CHECK(mpu9250_mag_init(mpu9250_handle));

	/******************************************************************************************
	 * GYRO & ACCEL
	 ******************************************************************************************/
	mpu9250_handle->data.acc_g_factor_initialized = 0;
	mpu9250_handle->data.speed_if[X_POS] = 0.0f;
	mpu9250_handle->data.speed_if[Y_POS] = 0.0f;
	mpu9250_handle->data.speed_if[Z_POS] = 0.0f;
	mpu9250_handle->data.vertical_acc_offset = 0.0f;

    // set Configuration Register
//	printf("MPU9250: Gyro bandwidth 184Hz\n");
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01)); // gyro bandwidth 184Hz
	printf("MPU9250: Gyro bandwidth 184Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01));

    // set Gyro Configuration Register
	printf("MPU9250: Gyro +-250deg/sec\n");
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_1000DPS));

    // set Acc Conf1 Register
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_8G));

    // set Acc Conf2 Register
	printf("MPU9250: Accel bandwidth 460Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG_2, 0x00));

    // set Clock Divider
	printf("MPU9250: Data rate 500Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_SMPLRT_DIV, 0x01));
    mpu9250_handle->data_rate = 500;

    // set PwrMgmt2 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_2, 0x00));

    // set PwrMgmt1 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_1, 0x01)); // clock, no cycle

    // set Int Status Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_STATUS, 0x00)); // reset all interrupts?

	printf("MPU9250: Enable Interrupts\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_ENABLE, 0x01)); // data ready int

	// prepare GPIO Interrupt
	printf("MPU9250: Gpio interrupt pin [%d]\n", mpu9250_handle->int_pin);
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_down_en=1,
        .pin_bit_mask=(1<<mpu9250_handle->int_pin)
    };
    gpio_config(&io_conf);
	printf("MPU9250: Gpio interrupt pin configured\n");

	printf("MPU9250: Configuring gpio interrupts ....\n");
	ESP_ERROR_CHECK(gpio_set_intr_type(mpu9250_handle->int_pin, GPIO_PIN_INTR_POSEDGE));
	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
	ESP_ERROR_CHECK(gpio_isr_handler_add(mpu9250_handle->int_pin, mpu9250_isr, (void*)mpu9250_handle));
	printf("MPU9250: Gpio interrupts configured ..\n");

    // Init Accel
	printf("MPU9250: Init Accel\n");
    ESP_ERROR_CHECK(mpu9250_acc_init(mpu9250_handle));

    // Init Gyro
	printf("MPU9250: Init Gyro\n");
    ESP_ERROR_CHECK(mpu9250_gyro_init(mpu9250_handle));

    return ESP_OK;
}

esp_err_t mpu9250_load_int_status(mpu9250_handle_t mpu9250_handle) {
	return mpu9250_read8(mpu9250_handle, MPU9250_INT_STATUS, &((mpu9250_handle_t)mpu9250_handle)->int_status);
}

esp_err_t mpu9250_load_whoami(mpu9250_handle_t mpu9250_handle) {
	esp_err_t ret = ESP_FAIL;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));        //Zero out the transaction
	t.length = 8;                    //Transaction length is in bits.
	t.addr = (MPU9250_READ_FLAG | MPU9250_WHO_AM_I);
	t.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(mpu9250_handle->device_handle, &t);  //Transmit!
	mpu9250_handle->whoami = (ret == ESP_OK ? t.rx_data[0] : 0);
	return ret;
}

esp_err_t mpu9250_test_connection(mpu9250_handle_t mpu9250_handle) {
	mpu9250_load_whoami(mpu9250_handle);
	return (mpu9250_handle->whoami == MPU9250_ID ? ESP_OK : ESP_FAIL);

}
esp_err_t mpu9250_load_raw_data(mpu9250_handle_t mpu9250_handle) {
	uint8_t buff[26];
	mpu9250_handle->data.timestamp = (uint32_t)(esp_timer_get_time()/1000);
	esp_err_t ret = mpu9250_read_buff(mpu9250_handle, MPU9250_ACCEL_XOUT_H, buff, 26*8);
	mpu9250_handle->data.raw_data.data_s_xyz.accel_data_x = ((buff[0] << 8) | buff[1]);
	mpu9250_handle->data.raw_data.data_s_xyz.accel_data_y = ((buff[2] << 8) | buff[3]);
	mpu9250_handle->data.raw_data.data_s_xyz.accel_data_z = ((buff[4] << 8) | buff[5]);
	mpu9250_handle->data.raw_data.data_s_xyz.temp_data = ((buff[6] << 8) | buff[7]);
	mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_x = ((buff[8] << 8) | buff[9]);
	mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_y = ((buff[10] << 8) | buff[11]);
	mpu9250_handle->data.raw_data.data_s_xyz.gyro_data_z = ((buff[12] << 8) | buff[13]);

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_cb_add(&mpu9250_handle->data.accel.cb[i], mpu9250_handle->data.raw_data.data_s_vector.accel[i]);
		mpu9250_cb_add(&mpu9250_handle->data.gyro.cb[i], mpu9250_handle->data.raw_data.data_s_vector.gyro[i]);
	}


	// read mag data
	for(uint8_t i = 0; i<16; i++) {
		buff[i] = 0;
	}
	ret = mpu9250_read_buff(mpu9250_handle, MPU9250_EXT_SENS_DATA_00, buff, 16*8);

	mpu9250_handle->data.mag.drdy = (buff[0] & 0x01) & ~((buff[7] & 0x08) >> 3); // drdy true e hofl false
	if(mpu9250_handle->data.mag.drdy) {
		mpu9250_handle->data.raw_data.data_s_vector.mag[X_POS] = ((buff[2] << 8) | buff[1]);
		mpu9250_handle->data.raw_data.data_s_vector.mag[Y_POS] = ((buff[4] << 8) | buff[3]);
		mpu9250_handle->data.raw_data.data_s_vector.mag[Z_POS] = ((buff[6] << 8) | buff[5]);
	}
	return ret;
}
/*
> applico il filtro seguente ai dati MAG,ACC calibrati
              MX           MY           MZ           AX            AY           AZ
[1,]  0.911713763 -0.079769789 -0.068227161 -0.067251825 -0.1400171500  0.002670831
[2,] -0.165199803  0.798955727 -0.002526119 -0.153190305 -0.2341330258  0.130293550
[3,] -0.026425360  0.052129760  0.882826268 -0.058521387 -0.0003191167 -0.101164079
[4,]  0.087247394  0.115087412  0.074371592  1.019536355  0.1307812869 -0.007049727
[5,]  0.002295771  0.043996185  0.007805331  0.008769458  1.1049553495 -0.012914788
[6,] -0.004600663 -0.009441748  0.008178032 -0.003655173 -0.0035476539  1.012830274
 */
esp_err_t mpu9250_filter_cal_data(mpu9250_handle_t mpu9250_handle) {
	float mX = mpu9250_handle->data.cal_data.data_s_xyz.mag_data_x;
	float mY = mpu9250_handle->data.cal_data.data_s_xyz.mag_data_y;
	float mZ = mpu9250_handle->data.cal_data.data_s_xyz.mag_data_z;
	float aX = mpu9250_handle->data.cal_data.data_s_xyz.accel_data_x;
	float aY = mpu9250_handle->data.cal_data.data_s_xyz.accel_data_y;
	float aZ = mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z;
	if(mpu9250_handle->data.mag.drdy) {
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_x = mX*0.911713763 + mY*(-0.079769789) + mZ*(-0.068227161) + aX*(-0.067251825) + aY*(-0.1400171500) +aZ*(0.002670831);
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_y = mX*(-0.165199803) + mY*(0.798955727) + mZ*(-0.002526119) + aX*(-0.153190305) + aY*(-0.2341330258) +aZ*(0.130293550);
		mpu9250_handle->data.cal_data.data_s_xyz.mag_data_z = mX*(-0.026425360) + mY*(0.052129760) + mZ*(0.882826268) + aX*(-0.058521387) + aY*(-0.0003191167) +aZ*(-0.101164079);
		mpu9250_handle->data.cal_data.data_s_xyz.accel_data_x = mX*(0.087247394) + mY*(0.115087412) + mZ*(0.074371592) + aX*(1.019536355) + aY*(0.1307812869) +aZ*(-0.007049727);
		mpu9250_handle->data.cal_data.data_s_xyz.accel_data_y = mX*(0.002295771) + mY*(0.043996185) + mZ*(0.007805331) + aX*(0.008769458) + aY*(1.1049553495) +aZ*(-0.012914788);
		mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z = mX*(-0.004600663) + mY*(-0.009441748) + mZ*(0.008178032) + aX*(-0.003655173) + aY*(-0.0035476539) +aZ*(1.012830274);
	}
	return ESP_OK;
}
int lcd_counter = 0;
esp_err_t mpu9250_load_cal_data(mpu9250_handle_t mpu9250_handle) {
#ifdef CONFIG_ESP_DATA_CAL
	ESP_ERROR_CHECK(mpu9250_gyro_load_cal_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_acc_load_cal_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_mag_load_cal_data(mpu9250_handle));
	mpu9250_handle->data.cal_data.data_s_xyz.temp_data = mpu9250_handle->data.raw_data.data_s_xyz.temp_data;
	//ESP_ERROR_CHECK(mpu9250_filter_cal_data(mpu9250_handle));
	lcd_counter++;
	lcd_counter %= 100;
	if(lcd_counter == 0) {
		float roll = -atan2(mpu9250_handle->data.cal_data.data_s_xyz.accel_data_y, sqrt(mpu9250_handle->data.cal_data.data_s_xyz.accel_data_x*mpu9250_handle->data.cal_data.data_s_xyz.accel_data_x + mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z*mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z));
		float pitch = -atan2(mpu9250_handle->data.cal_data.data_s_xyz.accel_data_x, sqrt(mpu9250_handle->data.cal_data.data_s_xyz.accel_data_y*mpu9250_handle->data.cal_data.data_s_xyz.accel_data_y + mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z*mpu9250_handle->data.cal_data.data_s_xyz.accel_data_z));
		printf("RP[%3.3f, %3.3f]\n", roll/PI*180, pitch/PI*180);
	}
#endif
	return ESP_OK;
}
esp_err_t mpu9250_load_data(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_load_cal_data(mpu9250_handle));
	return ESP_OK;
}
