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

	for(uint8_t i = 0; i < 3; i++) {
		mpu9250_cb_init(&mpu9250_handle->data.accel.cb[i]);
		mpu9250_cb_init(&mpu9250_handle->data.gyro.cb[i]);
		mpu9250_cb_init(&mpu9250_handle->data.gyro.cb_alfa[i]);
	}

    // set Configuration Register
//	printf("MPU9250: Gyro bandwidth 184Hz\n");
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01)); // gyro bandwidth 184Hz
	printf("MPU9250: Gyro bandwidth 184Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_CONFIG, 0x01));

    // set Gyro Configuration Register
	printf("MPU9250: Gyro +-250deg/sec\n");
	ESP_ERROR_CHECK(mpu9250_gyro_set_fsr(mpu9250_handle, INV_FSR_250DPS));

    // set Acc Conf1 Register
	ESP_ERROR_CHECK(mpu9250_acc_set_fsr(mpu9250_handle, INV_FSR_16G));

    // set Acc Conf2 Register
	printf("MPU9250: Accel bandwidth 460Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_ACCEL_CONFIG_2, 0x00));

    // set Clock Divider
	printf("MPU9250: Data rate 500Hz\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_SMPLRT_DIV, 0x01));
    mpu9250_handle->data_rate = 500;

//    // set Master Control
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_USER_CTRL, 0x20));
//
//    // I2C 400KHz
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_MST_CTRL, 0x0D));
//
//    // Reset AK8963 I2C Slave
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS));
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, AK8963_CNTL2)); // register where begin data transfer
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_DO, 0x01)); // reset ak8963
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, 0x81)); // send data
//    vTaskDelay(pdMS_TO_TICKS(10));
//
//    // Configure AK8963 Continuous Mode
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS));
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, AK8963_CNTL1)); // register where begin data transfer
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_DO, 0x12)); // value 16 bit continuous mode 1
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, 0x81));// send data
//    vTaskDelay(pdMS_TO_TICKS(5));
//
//    // Read Who Am I
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS|0x80));
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_REG, AK8963_INFO)); // register where begin data transfer
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_DO, 0x12)); // value 16 bit continuous mode 1
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_I2C_SLV0_CTRL, 0x81));// send data
//    vTaskDelay(pdMS_TO_TICKS(5));
//
//	printf("MPU9250: I2C BYPASS_EN\n");
//    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_PIN_CFG, 0x02));
//
    // set PwrMgmt2 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_2, 0x00));

    // set PwrMgmt1 Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_PWR_MGMT_1, 0x01)); // clock, no cycle

    // set Int Status Register
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_STATUS, 0x00)); // reset all interrupts?

	printf("MPU9250: Enable Interrupts\n");
    ESP_ERROR_CHECK(mpu9250_write8(mpu9250_handle, MPU9250_INT_ENABLE, 0x01)); // data ready int

    mpu9250_handle->data.attitude[X_POS] = 0.0f;
    mpu9250_handle->data.attitude[Y_POS] = 0.0f;
    mpu9250_handle->data.attitude[Z_POS] = 1.0f;

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
	return ret;
}

esp_err_t mpu9250_calc_gravity(mpu9250_handle_t mpu9250_handle) {
	double cx=cos(mpu9250_handle->data.gyro.rpy.xyz.x);
	double cy=cos(mpu9250_handle->data.gyro.rpy.xyz.y);
	double sx=sin(mpu9250_handle->data.gyro.rpy.xyz.x);
	double sy=sin(mpu9250_handle->data.gyro.rpy.xyz.y);
	// roll,pitch,yaw from gyro are axis rotation
	// then attitude vector rotation is inverse
	mpu9250_handle->data.attitude[X_POS] = -sy;
	mpu9250_handle->data.attitude[Y_POS] = cy*sx;
	mpu9250_handle->data.attitude[Z_POS] = cy*cx;

	return ESP_OK;
}
esp_err_t mpu9250_calc_rpy(mpu9250_handle_t mpu9250_handle) {

	// roll pitch fusion (accel + gyro)
	mpu9250_handle->data.gyro.rpy.xyz.x += 0.0025*(mpu9250_handle->data.accel.rpy.xyz.x - mpu9250_handle->data.gyro.rpy.xyz.x);
	mpu9250_handle->data.gyro.rpy.xyz.y += 0.0025*(mpu9250_handle->data.accel.rpy.xyz.y - mpu9250_handle->data.gyro.rpy.xyz.y);

	return ESP_OK;
}
esp_err_t mpu9250_update_state(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_calc_rpy(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_calc_gravity(mpu9250_handle));

	return ESP_OK;
}

esp_err_t mpu9250_load_data(mpu9250_handle_t mpu9250_handle) {
	ESP_ERROR_CHECK(mpu9250_load_raw_data(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_acc_update_state(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_gyro_update_state(mpu9250_handle));
	ESP_ERROR_CHECK(mpu9250_update_state(mpu9250_handle));

	return ESP_OK;
}

