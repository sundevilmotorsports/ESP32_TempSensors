// change_address.c
// Helper to change MLX90614 I2C address (EEPROM write) with SMBus PEC (CRC-8)
#include "mlx90614.h"
#include "esp_log.h"
#include <stdint.h>

static const char *TAG = "MLX90614_ADDR";

// CRC-8 (SMBus PEC) polynomial x^8 + x^2 + x + 1 = 0x07
static uint8_t crc8_pec(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Read a single byte from a RAM/EEPROM register using SMBus style (write reg then read 3 bytes low/high/PEC)
static esp_err_t read_word_with_pec(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t reg, uint16_t *out)
{
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
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "read_word i2c err %d", ret);
        return ret;
    }
    // Verify PEC
    uint8_t txbuf[4];
    txbuf[0] = (slave_addr << 1) | I2C_MASTER_WRITE; // address + W
    txbuf[1] = reg;
    txbuf[2] = (slave_addr << 1) | I2C_MASTER_READ; // address + R
    txbuf[3] = low;
    // PEC bytes for SMBus read should be computed over: Addr(W), Cmd, Addr(R), Data0, Data1
    uint8_t pec_in[5] = { txbuf[0], txbuf[1], txbuf[2], low, high };
    uint8_t calc = crc8_pec(pec_in, sizeof(pec_in));
    if (calc != pec) {
        ESP_LOGW(TAG, "PEC mismatch: calc=0x%02x got=0x%02x", calc, pec);
        // still return success but warn
    }
    *out = ((uint16_t)high << 8) | low;
    return ESP_OK;
}

// Write a word (low, high) to an EEPROM register using SMBus protocol with PEC.
// MLX90614 requires a special sequence in some cases; this implements the SMBus write with PEC.
static esp_err_t write_word_with_pec(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t reg, uint16_t value)
{
    uint8_t low = value & 0xFF;
    uint8_t high = (value >> 8) & 0xFF;
    // Build buffer for PEC computation: Addr(W), Cmd, Data0, Data1
    uint8_t pec_in[4] = { (uint8_t)((slave_addr << 1) | I2C_MASTER_WRITE), reg, low, high };
    uint8_t pec = crc8_pec(pec_in, sizeof(pec_in));

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, low, true);
    i2c_master_write_byte(cmd, high, true);
    i2c_master_write_byte(cmd, pec, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(300));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "write_word i2c err %d", ret);
    }
    return ret;
}

// Public helper to change address. current_addr and new_addr are 7-bit addresses.
esp_err_t mlx90614_change_address(i2c_port_t i2c_num, uint8_t current_addr, uint8_t new_addr)
{
    if (new_addr < 0x03 || new_addr > 0x77) {
        ESP_LOGE(TAG, "new_addr 0x%02x out of range", new_addr);
        return ESP_ERR_INVALID_ARG;
    }
    // Common MLX90614 breakouts store address in EEPROM register 0x2E (raw 8-bit), sometimes 0x0E.
    // We'll try 0x2E first, then 0x0E if verification fails.
    const uint8_t reg_candidates[2] = { 0x2E, 0x0E };
    uint16_t current_val;
    for (int i = 0; i < 2; ++i) {
        uint8_t reg = reg_candidates[i];
        esp_err_t r = read_word_with_pec(i2c_num, current_addr, reg, &current_val);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "Read reg 0x%02x -> 0x%04x", reg, current_val);
            // Value stored is often the 8-bit address in the low byte and 0x0000 in high byte
            // Prepare new word where low byte is new_addr and high byte preserved or zero
            uint16_t new_word = (current_val & 0xFF00) | (uint16_t)new_addr;
            r = write_word_with_pec(i2c_num, current_addr, reg, new_word);
            if (r != ESP_OK) {
                ESP_LOGW(TAG, "write to reg 0x%02x failed: %d", reg, r);
                continue; // try next candidate
            }
            // After writing EEPROM, MLX90614 may require time to complete internal write (~10-50 ms). Wait and verify.
            vTaskDelay(pdMS_TO_TICKS(200));
            // Try reading with new address
            uint16_t verify_val;
            esp_err_t v = read_word_with_pec(i2c_num, new_addr, reg, &verify_val);
            if (v == ESP_OK) {
                if ((verify_val & 0x00FF) == (new_addr & 0xFF)) {
                    ESP_LOGI(TAG, "Successfully changed address to 0x%02x (reg 0x%02x)", new_addr, reg);
                    return ESP_OK;
                } else {
                    ESP_LOGW(TAG, "Verification mismatch reg 0x%02x: read 0x%04x", reg, verify_val);
                }
            } else {
                ESP_LOGW(TAG, "Verification read failed at new addr 0x%02x: %d", new_addr, v);
            }
        } else {
            ESP_LOGW(TAG, "Could not read reg 0x%02x at addr 0x%02x: %d", reg, current_addr, r);
        }
    }
    ESP_LOGE(TAG, "Failed to change address to 0x%02x", new_addr);
    return ESP_FAIL;
}
