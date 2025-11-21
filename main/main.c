#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "mlx90614.h"

// Simple I2C scanner: scans 7-bit addresses 0x03..0x77 and logs any devices found.
static void i2c_scan(i2c_port_t i2c_num)
{
    ESP_LOGI("MAIN", "Starting I2C scan (0x03..0x77)...");
    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Try a quick write (no data) to probe for ACK
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI("MAIN", "I2C device found at 0x%02x", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW("MAIN", "No I2C devices detected on the bus.");
    } else {
        ESP_LOGI("MAIN", "I2C scan complete: %d device(s) found.", found);
    }
}

// Define MLX_CHANGE_ADDR_ON_BOOT to 1 to attempt a one-time address change on boot.
// Keep 0 to disable (recommended unless you are ready to change EEPROM).
// WARNING: EEPROM writes can be destructive. Ensure ONLY the target MLX90614
// is connected to the I2C bus before enabling this. Disconnect other devices.
#define MLX_CHANGE_ADDR_ON_BOOT 1
#define MLX_CURRENT_ADDR 0x5A
#define MLX_NEW_ADDR 0x5B



void app_main(void)
{

    // I2C configuration - change pins to match your wiring
    const i2c_port_t i2c_num = I2C_NUM_0;
    const int i2c_sda_pin = 1; // change as needed
    const int i2c_scl_pin = 2; // change as needed
    const int i2c_freq_hz = 100000;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda_pin,
        .scl_io_num = i2c_scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_freq_hz,
    };
    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);

    ESP_LOGI("MAIN", "I2C initialized on SDA=%d SCL=%d", i2c_sda_pin, i2c_scl_pin);

    // Run an I2C scan so you can confirm the MLX90614's current address before writing EEPROM.
    i2c_scan(i2c_num);

#if MLX_CHANGE_ADDR_ON_BOOT
    // One-shot address change with serial confirmation.
    // Safety: ensure only the target MLX90614 is connected to the I2C bus.
    ESP_LOGW("MAIN", "One-shot MLX90614 EEPROM address change is ENABLED but requires serial confirmation.");
    ESP_LOGI("MAIN", "To proceed, focus your serial monitor and type 'Y' (uppercase) within 30 seconds.");
    uint8_t c = 0;
    int read = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(30000));
    if (read > 0 && (c == 'Y' || c == 'y')) {
        ESP_LOGW("MAIN", "Confirmed. Attempting MLX90614 address change from 0x%02x to 0x%02x", MLX_CURRENT_ADDR, MLX_NEW_ADDR);
        esp_err_t change_res = mlx90614_change_address(i2c_num, MLX_CURRENT_ADDR, MLX_NEW_ADDR);
        if (change_res == ESP_OK) {
            ESP_LOGI("MAIN", "Address change successful. Verifying by reading object temp at new address...");
            float new_temp;
            esp_err_t vr = mlx90614_read_object_temp_at(i2c_num, MLX_NEW_ADDR, &new_temp);
            if (vr == ESP_OK) {
                ESP_LOGI("MAIN", "Verified read at 0x%02x: Object Temp %.2f C", MLX_NEW_ADDR, new_temp);
            } else {
                ESP_LOGW("MAIN", "Verification read at 0x%02x failed: %d", MLX_NEW_ADDR, vr);
            }
        } else {
            ESP_LOGE("MAIN", "Address change failed: %d", change_res);
        }
    } else {
        ESP_LOGI("MAIN", "Address change not confirmed or timed out; skipping EEPROM write.");
    }
#endif

    float to = 0.0f, ta = 0.0f;
    while (1) {
        if (mlx90614_read_object_temp(i2c_num, &to) == ESP_OK) {
            ESP_LOGI("MLX", "Object Temp: %.2f C", to);   ;
        } else {
            ESP_LOGW("MLX", "Failed to read object temp");
        }

        if (mlx90614_read_ambient_temp(i2c_num, &ta) == ESP_OK) {
            ESP_LOGI("MLX", "Ambient Temp: %.2f C", ta);
        } else {
            ESP_LOGW("MLX", "Failed to read ambient temp");
        }

        // Example write - commented out by default because EEPROM writes require care.
        // uint16_t example_val = 0x0000;
        // mlx90614_write_raw_register(i2c_num, 0x2E, example_val);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
