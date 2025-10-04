#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "mlx90614.h"



void app_main(void)
{

    // I2C configuration - change pins to match your wiring
    const i2c_port_t i2c_num = I2C_NUM_0;
    const int i2c_sda_pin = 21; // change as needed
    const int i2c_scl_pin = 22; // change as needed
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

    float to = 0.0f, ta = 0.0f;
    while (1) {
        if (mlx90614_read_object_temp(i2c_num, &to) == ESP_OK) {
            ESP_LOGI("MLX", "Object Temp: %.2f C", to);
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
