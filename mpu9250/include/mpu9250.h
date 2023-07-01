#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <esp_err.h>
#include <driver/spi_master.h>
#include "freertos/task.h"
#include <mpu9250_utility.h>
#define SDRONE_GRAVITY_ACCELERATION 9.80665f

#define MPU9250_ID 0x71
#define X_POS 0
#define Y_POS 1
#define Z_POS 2

enum mpu9250_register {
	MPU9250_SELF_TEST_X_GYRO =  0x00,
	MPU9250_SELF_TEST_Y_GYRO =  0x01,
	MPU9250_SELF_TEST_Z_GYRO =  0x02,
	MPU9250_SELF_TEST_X_ACCEL = 0x0D,
	MPU9250_SELF_TEST_Y_ACCEL = 0x0E,
	MPU9250_SELF_TEST_Z_ACCEL = 0x0F,
	MPU9250_XG_OFFSET_H =       0x13,
	MPU9250_XG_OFFSET_L =       0x14,
	MPU9250_YG_OFFSET_H =       0x15,
	MPU9250_YG_OFFSET_L =       0x16,
	MPU9250_ZG_OFFSET_H =       0x17,
	MPU9250_ZG_OFFSET_L =       0x18,
	MPU9250_SMPLRT_DIV =        0x19,
	MPU9250_CONFIG =            0x1A,
	MPU9250_GYRO_CONFIG =       0x1B,
	MPU9250_ACCEL_CONFIG =      0x1C,
	MPU9250_ACCEL_CONFIG_2 =    0x1D,
	MPU9250_LP_ACCEL_ODR =      0x1E,
	MPU9250_WOM_THR =           0x1F,
	MPU9250_FIFO_EN =           0x23,
	MPU9250_I2C_MST_CTRL =      0x24,
	MPU9250_I2C_SLV0_ADDR =     0x25,
	MPU9250_I2C_SLV0_REG =      0x26,
	MPU9250_I2C_SLV0_CTRL =     0x27,
	MPU9250_I2C_SLV1_ADDR =     0x28,
	MPU9250_I2C_SLV1_REG =      0x29,
	MPU9250_I2C_SLV1_CTRL =     0x2A,
	MPU9250_I2C_SLV2_ADDR =     0x2B,
	MPU9250_I2C_SLV2_REG =      0x2C,
	MPU9250_I2C_SLV2_CTRL =     0x2D,
	MPU9250_I2C_SLV3_ADDR =     0x2E,
	MPU9250_I2C_SLV3_REG =      0x2F,
	MPU9250_I2C_SLV3_CTRL =     0x30,
	MPU9250_I2C_SLV4_ADDR =     0x31,
	MPU9250_I2C_SLV4_REG =      0x32,
	MPU9250_I2C_SLV4_DO =       0x33,
	MPU9250_I2C_SLV4_CTRL =     0x34,
	MPU9250_I2C_SLV4_DI =       0x35,
	MPU9250_I2C_MST_STATUS =    0x36,
	MPU9250_INT_PIN_CFG =       0x37,
	MPU9250_INT_ENABLE =        0x38,
	MPU9250_INT_STATUS =        0x3A,
	MPU9250_ACCEL_XOUT_H =      0x3B,
	MPU9250_ACCEL_XOUT_L =      0x3C,
	MPU9250_ACCEL_YOUT_H =      0x3D,
	MPU9250_ACCEL_YOUT_L =      0x3E,
	MPU9250_ACCEL_ZOUT_H =      0x3F,
	MPU9250_ACCEL_ZOUT_L =      0x40,
	MPU9250_TEMP_OUT_H =        0x41,
	MPU9250_TEMP_OUT_L =        0x42,
	MPU9250_GYRO_XOUT_H =       0x43,
	MPU9250_GYRO_XOUT_L =       0x44,
	MPU9250_GYRO_YOUT_H =       0x45,
	MPU9250_GYRO_YOUT_L =       0x46,
	MPU9250_GYRO_ZOUT_H =       0x47,
	MPU9250_GYRO_ZOUT_L =       0x48,
	MPU9250_EXT_SENS_DATA_00 =  0x49,
	MPU9250_EXT_SENS_DATA_01 =  0x4A,
	MPU9250_EXT_SENS_DATA_02 =  0x4B,
	MPU9250_EXT_SENS_DATA_03 =  0x4C,
	MPU9250_EXT_SENS_DATA_04 =  0x4D,
	MPU9250_EXT_SENS_DATA_05 =  0x4E,
	MPU9250_EXT_SENS_DATA_06 =  0x4F,
	MPU9250_EXT_SENS_DATA_07 =  0x50,
	MPU9250_EXT_SENS_DATA_08 =  0x51,
	MPU9250_EXT_SENS_DATA_09 =  0x52,
	MPU9250_EXT_SENS_DATA_10 =  0x53,
	MPU9250_EXT_SENS_DATA_11 =  0x54,
	MPU9250_EXT_SENS_DATA_12 =  0x55,
	MPU9250_EXT_SENS_DATA_13 =  0x56,
	MPU9250_EXT_SENS_DATA_14 =  0x57,
	MPU9250_EXT_SENS_DATA_15 =  0x58,
	MPU9250_EXT_SENS_DATA_16 =  0x59,
	MPU9250_EXT_SENS_DATA_17 =  0x5A,
	MPU9250_EXT_SENS_DATA_18 =  0x5B,
	MPU9250_EXT_SENS_DATA_19 =  0x5C,
	MPU9250_EXT_SENS_DATA_20 =  0x5D,
	MPU9250_EXT_SENS_DATA_21 =  0x5E,
	MPU9250_EXT_SENS_DATA_22 =  0x5F,
	MPU9250_EXT_SENS_DATA_23 =  0x60,
	MPU9250_I2C_SLV0_DO =       0x63,
	MPU9250_I2C_SLV1_DO =       0x64,
	MPU9250_I2C_SLV2_DO =       0x65,
	MPU9250_I2C_SLV3_DO =       0x66,
	MPU9250_I2C_MST_DELAY_CTRL =0x67,
	MPU9250_SIGNAL_PATH_RESET = 0x68,
	MPU9250_MOT_DETECT_CTRL =   0x69,
	MPU9250_USER_CTRL =         0x6A,
	MPU9250_PWR_MGMT_1 =        0x6B,
	MPU9250_PWR_MGMT_2 =        0x6C,
	MPU9250_FIFO_COUNTH =       0x72,
	MPU9250_FIFO_COUNTL =       0x73,
	MPU9250_FIFO_R_W =          0x74,
	MPU9250_WHO_AM_I =          0x75,
	MPU9250_XA_OFFSET_H =       0x77,
	MPU9250_XA_OFFSET_L =       0x78,
	MPU9250_YA_OFFSET_H =       0x7A,
	MPU9250_YA_OFFSET_L =       0x7B,
	MPU9250_ZA_OFFSET_H =       0x7D,
	MPU9250_ZA_OFFSET_L =       0x7E
};
#define MPU9250_WHO_AM_I_RESULT 0x70

/*********************************
*********** Gyroscope ************
*********************************/
enum gyro_config_bits {
	GYRO_CONFIG_FCHOICE_B = 0,
	GYRO_CONFIG_GYRO_FS_SEL = 3,
	GYRO_CONFIG_ZGYRO_CTEN = 5,
	GYRO_CONFIG_YGYRO_CTEN = 6,
	GYRO_CONFIG_XGYRO_CTEN = 7,
};

/* Gyro Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

#define MPU9250_GYRO_FSR_MASK     0xE7
#define MPU9250_GYRO_FSR_LBIT     0x03
#define MPU9250_GYRO_FCHOICE_MASK 0x03

/*********************************
********* Accelerometer **********
*********************************/
enum accel_config_bit {
	ACCEL_CONFIG_ACCEL_FS_SEL = 3,
	ACCEL_CONFIG_AZ_ST_EN = 5,
	ACCEL_CONFIG_AY_ST_EN = 6,
	ACCEL_CONFIG_AX_ST_EN = 7,
};
#define MPU9250_ACC_FSR_MASK    0xE7
#define MPU9250_ACC_FSR_LBIT    0x03


enum accel_config_2_bits {
	ACCEL_CONFIG_2_A_DLPFCFG = 0,
	ACCEL_CONFIG_2_ACCEL_FCHOICE_B = 3,
};

/* Accel Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};
/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
    INV_LPA_0_3125HZ,
    INV_LPA_0_625HZ,
    INV_LPA_1_25HZ,
    INV_LPA_2_5HZ,
    INV_LPA_5HZ,
    INV_LPA_10HZ,
    INV_LPA_20HZ,
    INV_LPA_40HZ,
    INV_LPA_80HZ,
    INV_LPA_160HZ,
    INV_LPA_320HZ,
    INV_LPA_640HZ
};

/*********************************
********* Magnetometer ***********
*********************************/
typedef enum  {
    INV_MAG_PRECISION_14_BITS = 0,
    INV_MAG_PRECISION_16_BITS
} mag_precision_e;

/*********************************
******* Power Management *********
*********************************/
enum pwr_mgmt_1_bits {
	PWR_MGMT_1_CLKSEL = 0,
	PWR_MGMT_1_PD_PTAT = 3,
	PWR_MGMT_1_GYRO_STANDBY = 4,
	PWR_MGMT_1_CYCLE = 5,
	PWR_MGMT_1_SLEEP = 6,
	PWR_MGMT_1_H_RESET = 7
};

enum pwr_mgmt_2_bits {
	PWR_MGMT_2_DISABLE_ZG = 0,
	PWR_MGMT_2_DISABLE_YG = 1,
	PWR_MGMT_2_DISABLE_XG = 2,
	PWR_MGMT_2_DISABLE_ZA = 3,
	PWR_MGMT_2_DISABLE_YA = 4,
	PWR_MGMT_2_DISABLE_XA = 5,
};

/*********************************
*********** Interrupts ***********
*********************************/
enum interrupt_status_bits {
	INT_STATUS_RAW_DATA_RDY_INT = 0,
	INT_STATUS_RAW_DMP_INT = 1,
	INT_STATUS_FSYNC_INT = 3,
	INT_STATUS_FIFO_OVERFLOW_INT = 4,
	INT_STATUS_WOM_INT = 6,
};

enum int_enable_bits {
	INT_ENABLE_RAW_RDY_EN = 0,
	INT_ENABLE_FSYNC_INT_EN = 3,
	INT_ENABLE_FIFO_OVERFLOW_EN = 4,
	INT_ENABLE_WOM_EN = 6,
};

enum int_pin_cfg_bits {
	INT_PIN_CFG_BYPASS_EN = 1,
	INT_PIN_CFG_FSYNC_INT_MODE_EN = 2,
	INT_PIN_CFG_ACTL_FSYNC = 3,
	INT_PIN_CFG_INT_ANYRD_2CLEAR = 4,
	INT_PIN_CFG_LATCH_INT_EN = 5,
	INT_PIN_CFG_OPEN = 6,
	INT_PIN_CFG_ACTL = 7,
};
#define INT_PIN_CFG_INT_MASK 0xF0

/*********************************
************* Compass ************
*********************************/
enum ak8963_register {
	AK8963_WIA = 0x0,
	AK8963_INFO = 0x1,
	AK8963_ST1 = 0x2,
	AK8963_HXL = 0x3,
	AK8963_HXH = 0x4,
	AK8963_HYL = 0x5,
	AK8963_HYH = 0x6,
	AK8963_HZL = 0x7,
	AK8963_HZH = 0x8,
	AK8963_ST2 = 0x9,
	AK8963_CNTL1 = 0xA,
	AK8963_CNTL2 = 0xB,
	AK8963_ASTC = 0xC,
	AK8963_TS1 = 0xD,
	AK8963_TS2 = 0xE,
	AK8963_I2CDIS = 0xF,
	AK8963_ASAX = 0x10,
	AK8963_ASAY = 0x11,
	AK8963_ASAZ = 0x12,
	AK8963_RSV = 0x13
};

#define MAG_CTRL_OP_MODE_MASK 0xF
#define AK8963_ST1_DRDY_BIT 0
#define AK8963_WHO_AM_I_RESULT 0x48
#define AK8963_ADDRESS 0x0C
#define AK8963_PWR_DOWN 0x00
#define I2C_SLV_EN 0x80
#define MPU9250_I2C_READ_FLAG 0x80
#define AK8963_RESET 0x01
#define AK8963_SINGLE_MEASUREMENT 0x01
#define I2C_MST_EN 0x20
#define I2C_MST_CLK 0x0D
#define H_RESET 0x80
#define CLKSEL_PLL 0x01
#define AK8963_FUSE_ROM 0x0F
#define AK8963_PRECISION_MASK 0x10
#define AK8963_MODE_2 0x06

/*********************************
********* Low Pass Filter ********
*********************************/
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};


/*********************************
********* Clock Sources **********
*********************************/
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

/* RPY */
typedef mpu9250_double_3d_t mpu9250_rpy_t;

typedef struct {
    float offsets[3];
    float factors[3][3];
} mpu9250_cal_data_t;

typedef mpu9250_float_3d_t mpu9250_scale_factors_t;

typedef struct {
    mpu9250_uint8_3d_t asa;
    mpu9250_scale_factors_t factors; // factory factors
} mpu9250_mag_cal_data_t;

#define PI_2 6.283185307f
#define PI 3.141592654f
#define PI_HALF 1.570796327f

/********************************************************************************************************************
********** MPU9250 API **********************************************************************************************
********************************************************************************************************************/
typedef union {
   struct {
     int16_t accel[3];
     int16_t temp;
     int16_t gyro[3];
     int16_t mag[3];
     int16_t ext[9];
   } data_s_vector;
   struct {
     int16_t accel_data_x;
     int16_t accel_data_y;
     int16_t accel_data_z;
     int16_t temp_data;
     int16_t gyro_data_x;
     int16_t gyro_data_y;
     int16_t gyro_data_z;
     int16_t mag_data_x;
     int16_t mag_data_y;
     int16_t mag_data_z;
     int16_t ext_data[9];
   } data_s_xyz;
} mpu9250_raw_data_t;
typedef mpu9250_raw_data_t* mpu9250_raw_data_buff_t;

typedef union {
   struct {
     float accel[3];
     float temp;
     float gyro[3];
     float mag[3];
   } data_s_vector;
   struct {
     float accel_data_x;
     float accel_data_y;
     float accel_data_z;
     float temp_data;
     float gyro_data_x;
     float gyro_data_y;
     float gyro_data_z;
     float mag_data_x;
     float mag_data_y;
     float mag_data_z;
   } data_s_xyz;
} mpu9250_calibrated_data_t;
typedef mpu9250_calibrated_data_t* mpu9250_calibrated_data_buff_t;

/*********************************
********* ACCELEROMETER **********
*********************************/
typedef struct mpu9250_accel_s {
    mpu9250_cal_data_t cal; // calibration data
	uint8_t fsr;
    uint16_t lsb;
} mpu9250_accel_t;

/*********************************
*********** GYROSCOPE ************
*********************************/
typedef struct mpu9250_gyro_s {
    int16_t device_offsets[3]; // device offset (not my calibration params)
	uint8_t fsr;
    float lsb;
} mpu9250_gyro_t;

typedef uint8_t mpu9250_int_status_t;

/*********************************
********* MAGNETOMETER ***********
*********************************/
typedef struct mpu9250_mag_s {
	mpu9250_mag_cal_data_t factory_cal; // factory mag calibration data
    mpu9250_cal_data_t cal; // my calibration data
	mag_precision_e precision;
    float precision_factor;
    uint8_t drdy;
} mpu9250_mag_t;

/*********************************
******** MPU9250 HANDLE **********
*********************************/
typedef struct {
	uint32_t timestamp;
    mpu9250_raw_data_t raw_data;
    mpu9250_calibrated_data_t cal_data;
    uint32_t nvs_cal_data;
    mpu9250_accel_t accel;
    mpu9250_gyro_t gyro;
    mpu9250_mag_t mag;
} mpu9250_data_t;

typedef struct mpu9250_init_s {
	spi_bus_config_t buscfg;
	spi_device_interface_config_t devcfg;
	spi_device_handle_t device_handle;
	TaskHandle_t data_ready_task_handle;

	// int config and status
	int int_pin;
    uint8_t int_status;

    // MPU9250 id
    uint8_t whoami;
    mpu9250_data_t data;
    uint16_t data_rate;
} mpu9250_init_t;

typedef mpu9250_init_t* mpu9250_handle_t;

#define MPU9250_READ_FLAG 0x80

/* Public Methods */
/* Set up APIs */
esp_err_t mpu9250_init(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_test_connection(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_whoami(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_int_status(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_raw_data(mpu9250_handle_t mpu9250_handle);
esp_err_t mpu9250_load_data(mpu9250_handle_t mpu9250_handle);

#endif // _MPU9250_H_
