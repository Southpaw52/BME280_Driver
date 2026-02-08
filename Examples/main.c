/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bme_280.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

bme280_raw_t raw;
double T, P, H;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* Private user code ---------------------------------------------------------*/
int main(void)
{
    HAL_Init();                  // Initialize HAL Library
    SystemClock_Config();        // Configure the system clock
    MX_GPIO_Init();              // Initialize GPIO
    MX_I2C1_Init();              // Initialize I2C1

    HAL_StatusTypeDef st;

    // Initialize BME280 sensor
    st = bme280_init();         
    if (st != HAL_OK)
    {
        Error_Handler();          // Stop if initialization fails
    }

    // Trigger a single measurement in FORCED mode
    st = bme280_force_measure();
    if (st != HAL_OK)
    {
        Error_Handler();          // Stop if write fails
    }

    while (1)
    {
        // Wait until measurement is complete
        while (bme280_is_measuring());

        // Read raw sensor data
        st = bme280_read_raw(&raw); 
        if (st != HAL_OK)
        {
            Error_Handler();
        }

        // Convert raw data to human-readable values
        T = bme280_compensate_T_double(raw.adc_T);  
        P = bme280_compensate_P_double(raw.adc_P); 
        H = bme280_compensate_H_double(raw.adc_H);  

        HAL_Delay(500);  // Delay between measurements
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* I2C1 Initialization Function */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Stay here if error occurs
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    // Optional: report assert failure
}
#endif
