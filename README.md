MLX90614 (IR temperature sensor) usage

Wiring (typical):

- MLX90614 VIN -> 3.3V
- MLX90614 GND -> GND
- MLX90614 SDA -> ESP32 SDA pin (default in example: GPIO21)
- MLX90614 SCL -> ESP32 SCL pin (default in example: GPIO22)

Notes:

- The example in `main/main.c` uses the simple driver `main/mlx90614.c` and prints object and ambient
  temperature every 2 seconds. Adjust `i2c_sda_pin` and `i2c_scl_pin` inside `main.c` to match your board.
- EEPROM writes to MLX90614 are potentially destructive and require following the device's write sequence and PEC checks.
  The example includes a helper `mlx90614_write_raw_register()` but it's commented out by default.
- Build and flash with ESP-IDF: set the target with `idf.py set-target esp32s3` then run `idf.py -p <PORT> flash monitor`.
