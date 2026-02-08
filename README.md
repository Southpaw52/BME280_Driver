# 🌡️ BME280 Driver (C)

 Lightweight and platform-independent C driver for the **Bosch BME280** environmental sensor  
 (temperature, pressure, and humidity).  
 Tested on **STM32F407** with custom low-level drivers.

---

## 📌 Features
| Feature | Description |
|----------|--------------|
| Communication | I²C (user-provided low-level read/write functions) |
| Temperature | Read ambient temperature (°C) |
| Pressure | Read air pressure (Pa) |
| Humidity | Read relative humidity (%RH) |
| Configuration | Oversampling, IIR filter, standby time, and power modes |

---

## 🧠 Usage Example
```c
/* Platform-specific I²C read/write functions must be provided by the user */ 
#include "bme280.h"

bme280_dev_t bme = {
    .dev_addr  = BME280_I2C_ADDR_PRIM,   // 0x76 (default) or BME280_I2C_ADDR_SEC (0x77)
    .osr_t     = BME280_OSR_T_2X,
    .osr_p     = BME280_OSR_P_4X,
    .osr_h     = BME280_OSR_H_1X,
    .filter    = BME280_FILTER_4,
    .standby   = BME280_STBY_1000_MS,
    .mode      = BME280_MODE_NORMAL,
    .i2c_read  = platform_i2c_read,
    .i2c_write = platform_i2c_write,
    .delay_ms  = platform_delay_ms
};

bme280_data_t data;

if (bme280_init(&bme) == BME280_OK) {
    while (1) {
        if (bme280_read_all(&bme, &data) == BME280_OK) {
            printf("Temp=%.2f °C | Pressure=%.2f Pa | Humidity=%.2f %%RH\n",
                   data.temperature_c,
                   data.pressure_pa,
                   data.humidity_rh);
        }
        platform_delay_ms(1000);
    }
}
