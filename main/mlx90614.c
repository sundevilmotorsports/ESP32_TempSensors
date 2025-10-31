// Basic MLX90614 driver implementation
#include "mlx90614.h"
#include "esp_log.h"
#include <stdint.h>

static const char *TAG = "MLX90614";
// Default 7-bit I2C address for many MLX90614 modules
#define MLX90614_I2C_ADDR 0x5A

// Read two bytes from a RAM register (read command: 0x07 / 0x06?), but MLX90614 uses SMBus read protocol.
// We'll use the SMBus read by writing the register address then reading two bytes (low, high) and PEC (ignored).

static esp_err_t mlx_read_word(i2c_port_t i2c_num, uint8_t reg, uint16_t *out)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MLX90614_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MLX90614_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    uint8_t low = 0, high = 0, pec = 0;
    i2c_master_read_byte(cmd, &low, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &high, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &pec, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        *out = ((uint16_t)high << 8) | low;
    } else {
        ESP_LOGW(TAG, "i2c read word failed: %d", ret);
    }
    return ret;
}

static esp_err_t mlx_write_word(i2c_port_t i2c_num, uint8_t reg, uint16_t value)
{
    // MLX90614 EEPROM write requires PEC and a special sequence. Many modules don't support direct writes via simple SMBus;
    // provide a raw write helper but warn the user.
    ESP_LOGW(TAG, "Writing EEPROM may require special unlocking sequence; using basic write (may fail)");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MLX90614_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    uint8_t low = value & 0xFF;
    uint8_t high = (value >> 8) & 0xFF;
    i2c_master_write_byte(cmd, low, true);
    i2c_master_write_byte(cmd, high, true);
    // Not computing PEC here
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mlx90614_read_raw_register(i2c_port_t i2c_num, uint8_t reg, uint16_t *value)
{
    return mlx_read_word(i2c_num, reg, value);
}

esp_err_t mlx90614_write_raw_register(i2c_port_t i2c_num, uint8_t reg, uint16_t value)
{
    return mlx_write_word(i2c_num, reg, value);
}

// Runtime address variants
esp_err_t mlx90614_read_raw_register_at(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t reg, uint16_t *value)
{
    // Inline similar logic to mlx_read_word but using slave_addr
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true);
    uint8_t low = 0, high = 0, pec = 0;
    i2c_master_read_byte(cmd, &low, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &high, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &pec, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        *value = ((uint16_t)high << 8) | low;
    } else {
        ESP_LOGW(TAG, "i2c read word failed: %d", ret);
    }
    return ret;
}

esp_err_t mlx90614_read_object_temp_at(i2c_port_t i2c_num, uint8_t slave_addr, float *temp_c)
{
    uint16_t raw;
    esp_err_t r = mlx90614_read_raw_register_at(i2c_num, slave_addr, 0x07, &raw);
    if (r != ESP_OK) return r;
    float temp_k = raw * 0.02f;
    *temp_c = temp_k - 273.15f;
    return ESP_OK;
}

esp_err_t mlx90614_read_ambient_temp_at(i2c_port_t i2c_num, uint8_t slave_addr, float *temp_c)
{
    uint16_t raw;
    esp_err_t r = mlx90614_read_raw_register_at(i2c_num, slave_addr, 0x06, &raw);
    if (r != ESP_OK) return r;
    float temp_k = raw * 0.02f;
    *temp_c = temp_k - 273.15f;
    return ESP_OK;
}

esp_err_t mlx90614_read_object_temp(i2c_port_t i2c_num, float *temp_c)
{
    uint16_t raw;
    // Object temperature register is 0x07 for many MLX90614 variants
    esp_err_t r = mlx_read_word(i2c_num, 0x07, &raw);
    if (r != ESP_OK) return r;
    // OS: value is in 0.02K per LSB
    float temp_k = raw * 0.02f;
    *temp_c = temp_k - 273.15f;
    return ESP_OK;
}

esp_err_t mlx90614_read_ambient_temp(i2c_port_t i2c_num, float *temp_c)
{
    uint16_t raw;
    // Ambient temperature register is 0x06
    esp_err_t r = mlx_read_word(i2c_num, 0x06, &raw);
    if (r != ESP_OK) return r;
    float temp_k = raw * 0.02f;
    *temp_c = temp_k - 273.15f;
    return ESP_OK;
}
