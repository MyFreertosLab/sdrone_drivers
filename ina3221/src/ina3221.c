/*
 *
 */

#include <ina3221.h>
#include <string.h>

#define INA3221_ACK_CHECK_EN 0x1
#define INA3221_ACK_VAL      0x0
#define INA3221_NACK_VAL     0x1

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
esp_err_t ina3221_init(ina3221_handle_t ina3221_handle) {

	// Set Config register to take into account the settings above
	uint16_t config =
	INA3221_CONFIG_CH1 |
	INA3221_CONFIG_CH2 |
	INA3221_CONFIG_CH3 |
	INA3221_CONFIG_VBUS_CT2 |
	INA3221_CONFIG_VSH_CT2 |
	INA3221_CONFIG_MODE_3 |
	INA3221_CONFIG_MODE_2 |
	INA3221_CONFIG_MODE_1;

	ESP_ERROR_CHECK(ina3221_write_register(ina3221_handle, INA3221_REG_CONFIG, config));
	return ESP_OK;
}

esp_err_t ina3221_write8(ina3221_handle_t ina3221_handle, uint8_t value) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, INA3221_ADDRESS | I2C_MASTER_WRITE,
			INA3221_ACK_CHECK_EN);
	i2c_master_write(cmd, &value, 1, INA3221_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(
			i2c_master_cmd_begin(ina3221_handle->i2c_master_port, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	return ESP_OK;
}

esp_err_t ina3221_write16(ina3221_handle_t ina3221_handle, uint16_t value) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, INA3221_ADDRESS | I2C_MASTER_WRITE,
			INA3221_ACK_CHECK_EN);
	i2c_master_write(cmd, (uint8_t*)(&value), 2, INA3221_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(
			i2c_master_cmd_begin(ina3221_handle->i2c_master_port, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	return ESP_OK;
}

esp_err_t ina3221_read16(ina3221_handle_t ina3221_handle, uint16_t *value) {
	uint8_t buff[3];
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, INA3221_ADDRESS | I2C_MASTER_READ,
			INA3221_ACK_CHECK_EN);
	i2c_master_read(cmd, (uint8_t*) buff, 2, INA3221_ACK_VAL);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(
			i2c_master_cmd_begin(ina3221_handle->i2c_master_port, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	*value = (buff[0] << 8) | buff[1];
	return ESP_OK;
}

esp_err_t ina3221_read_buff(ina3221_handle_t ina3221_handle, uint8_t* buff, uint8_t len) {
	uint8_t tmp[len+1];
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, INA3221_ADDRESS | I2C_MASTER_READ,
			INA3221_ACK_CHECK_EN);
	i2c_master_read(cmd, (uint8_t*) tmp, len, INA3221_ACK_VAL);
	i2c_master_stop(cmd);
	ESP_ERROR_CHECK(
			i2c_master_cmd_begin(ina3221_handle->i2c_master_port, cmd, 1000 / portTICK_RATE_MS));
	i2c_cmd_link_delete(cmd);
	memcpy(buff, tmp, len);
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t ina3221_read_register(ina3221_handle_t ina3221_handle,
		uint8_t address, uint16_t *value) {
	ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, address));
	ESP_ERROR_CHECK(ina3221_read16(ina3221_handle, value));
	return ESP_OK;
}

esp_err_t ina3221_write_register(ina3221_handle_t ina3221_handle,
		uint8_t address, uint16_t value) {
	ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, address));
	ESP_ERROR_CHECK(ina3221_write16(ina3221_handle, value));
	return ESP_OK;
}

esp_err_t ina3221_test_connection(ina3221_handle_t ina3221_handle) {
	uint16_t data_rd;
	ESP_ERROR_CHECK(
			ina3221_read_register(ina3221_handle, INA3221_REG_MANUFACTURER_ID, &data_rd));

	if (data_rd == INA3221_REG_MANUFACTURER_ID_VALUE) {
		return ESP_OK;
	}
	return ESP_FAIL;
}

esp_err_t ina3221_load_shunt_voltage(ina3221_handle_t ina3221_handle, uint8_t channel) {
	switch(channel) {
	case 0:
		ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, INA3221_REG_SHUNTVOLTAGE_1));
		break;
	case 1:
		ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, INA3221_REG_SHUNTVOLTAGE_2));
		break;
	case 2:
		ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, INA3221_REG_SHUNTVOLTAGE_3));
		break;
	default:
		return ESP_FAIL;
	}
	ESP_ERROR_CHECK(ina3221_read16(ina3221_handle, (uint16_t*)&ina3221_handle->raw_data.channel_data[channel].shunt_voltage));
	return ESP_OK;
}
esp_err_t ina3221_load_bus_voltage(ina3221_handle_t ina3221_handle, uint8_t channel) {
	switch(channel) {
	case 0:
		ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, INA3221_REG_BUSVOLTAGE_1));
		break;
	case 1:
		ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, INA3221_REG_BUSVOLTAGE_2));
		break;
	case 2:
		ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, INA3221_REG_BUSVOLTAGE_3));
		break;
	default:
		return ESP_FAIL;
	}
	ESP_ERROR_CHECK(ina3221_read16(ina3221_handle, (uint16_t*)&ina3221_handle->raw_data.channel_data[channel].bus_voltage));
	return ESP_OK;
}

esp_err_t ina3321_load_data(ina3221_handle_t ina3221_handle) {
	for(uint8_t channel = 0; channel < 3; channel++) {
		ESP_ERROR_CHECK(ina3221_load_shunt_voltage(ina3221_handle, channel));
		ESP_ERROR_CHECK(ina3221_load_bus_voltage(ina3221_handle, channel));
	}
	return ESP_OK;
}
