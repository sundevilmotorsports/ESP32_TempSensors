// Simple MLX90614 helper for ESP-IDF
#ifndef MLX90614_H
#define MLX90614_H

#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t mlx90614_read_object_temp(i2c_port_t i2c_num, float *temp_c);
esp_err_t mlx90614_read_ambient_temp(i2c_port_t i2c_num, float *temp_c);
esp_err_t mlx90614_read_raw_register(i2c_port_t i2c_num, uint8_t reg, uint16_t *value);
esp_err_t mlx90614_write_raw_register(i2c_port_t i2c_num, uint8_t reg, uint16_t value);

#endif // MLX90614_H
