// Simple MLX90614 helper for ESP-IDF
#ifndef MLX90614_H
#define MLX90614_H

#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t mlx90614_read_object_temp(i2c_port_t i2c_num, float *temp_c);
esp_err_t mlx90614_read_ambient_temp(i2c_port_t i2c_num, float *temp_c);
esp_err_t mlx90614_read_raw_register(i2c_port_t i2c_num, uint8_t reg, uint16_t *value);
esp_err_t mlx90614_write_raw_register(i2c_port_t i2c_num, uint8_t reg, uint16_t value);
// Change the 7-bit I2C slave address stored in MLX90614 EEPROM.
// new_addr: 7-bit address (0x03..0x77 typical). This operation is potentially destructive
// and requires the device to support EEPROM writes. Use with caution.
esp_err_t mlx90614_change_address(i2c_port_t i2c_num, uint8_t current_addr, uint8_t new_addr);

// Versions of read functions that accept a runtime slave address (7-bit). Useful for verification
// after changing EEPROM address without updating a compile-time define.
esp_err_t mlx90614_read_raw_register_at(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t reg, uint16_t *value);
esp_err_t mlx90614_read_object_temp_at(i2c_port_t i2c_num, uint8_t slave_addr, float *temp_c);
esp_err_t mlx90614_read_ambient_temp_at(i2c_port_t i2c_num, uint8_t slave_addr, float *temp_c);

#endif // MLX90614_H
