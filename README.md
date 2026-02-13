# 🌡️ BME280 Driver (C)

Lightweight and platform-independent C driver for the Bosch BME280 environmental sensor (temperature, pressure, and humidity).
Tested on STM32F407 using STM32 HAL I2C drivers.

---

## 📌 Features
| Feature | Description |
|----------|--------------|
| Communication | I²C (HAL-based low-level interface, user adaptable) |
| Temperature | Read environmental temperature (°C) |
| Pressure | Read air pressure (Pa) |
| Humidity | Read relative humidity (%RH) |
| Configuration | Oversampling, IIR filter, standby time, and power modes |

---

## 🧠 Usage Example
```c
/* Platform-specific I²C read/write functions must be provided by the user */
#include "main.h"
#include "bme280.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

bme280_raw_t raw;
double T, P, H;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

int main ()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    HAL_StatusTypeDef st;
    st = bme280_init();          
    st = bme280_force_measure(); 
    while (bme280_is_measuring()) {}  
    st = bme280_read_raw(&raw);  
    T = bme280_compensate_T_double(raw.adc_T);  
    P = bme280_compensate_P_double(raw.adc_P);  
    H = bme280_compensate_H_double(raw.adc_H);
}

