/*
 * bme_280.c
 *
 *  Created on: Jan 29, 2026
 *      Author: bahti
 */
#include "bme_280.h"

extern I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef *i2c1 = &hi2c1;
static bme280_calib_t cal;
static int32_t t_fine;

HAL_StatusTypeDef bme280_init(void)
{

	HAL_StatusTypeDef status;

		// 1. RESET
		uint8_t pData = BME280_SOFT_RESET_CMD;
		status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_RESET, I2C_MEMADD_SIZE_8BIT, &pData, 1, BME280_I2C_TIMEOUT_MS);
		if(status != HAL_OK)
		{
			return status;
		}

		// 2. CHIP ID OKUMA

		uint8_t read;
		status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_ID, I2C_MEMADD_SIZE_8BIT, &read, 1, BME280_I2C_TIMEOUT_MS);
		if(status != HAL_OK)
		{
			return status;
		}
		if(read != BME280_CHIP_ID)
		{
			return HAL_ERROR;
		}

		// 3. STATUS KONTROLÜ
		uint8_t status_reg = 1;
		while (status_reg & 0x01)
		{
			HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_STATUS, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, BME280_I2C_TIMEOUT_MS);
		}

		// 4. KALİBRASYON VERİLERİ OKUMA
		uint8_t buf1[BME280_CALIB_TP_LEN]; // 0x88..0xA1
		uint8_t buf2[BME280_CALIB_H_LEN];  // 0xE1..0xE7

		status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_CALIB_TP_START, I2C_MEMADD_SIZE_8BIT, buf1, sizeof(buf1), BME280_I2C_TIMEOUT_MS);
		if (status != HAL_OK)
		{
			return status;
		}

		status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_CALIB_H_START, I2C_MEMADD_SIZE_8BIT, buf2, sizeof(buf2), BME280_I2C_TIMEOUT_MS);
		if (status != HAL_OK)
		{
			return status;
		}

		/*
		 * 5. KALİBRASYON VERİLERİ
		 * BME280 verileri little-endian formatta gönderir --> before little-byte, after big-byte
		 * (LSB düşük adres, MSB yüksek adres)
		 */

		/* dig_T1 unsigned */
		cal.dig_T1 = (uint16_t)((buf1[1] << 8) | buf1[0]);

		/* dig_T2 signed */
		cal.dig_T2 = (int16_t)((buf1[3] << 8) | buf1[2]);

		/* dig_T3 signed */
		cal.dig_T3 = (int16_t)((buf1[5] << 8) | buf1[4]);

		/* dig_P1 unsigned */
		cal.dig_P1 = (uint16_t)((buf1[7] << 8) | buf1[6]);

		/* dig_P2..dig_P9 signed */
		cal.dig_P2 = (int16_t)((buf1[9]  << 8) | buf1[8]);
		cal.dig_P3 = (int16_t)((buf1[11] << 8) | buf1[10]);
		cal.dig_P4 = (int16_t)((buf1[13] << 8) | buf1[12]);
		cal.dig_P5 = (int16_t)((buf1[15] << 8) | buf1[14]);
		cal.dig_P6 = (int16_t)((buf1[17] << 8) | buf1[16]);
		cal.dig_P7 = (int16_t)((buf1[19] << 8) | buf1[18]);
		cal.dig_P8 = (int16_t)((buf1[21] << 8) | buf1[20]);
		cal.dig_P9 = (int16_t)((buf1[23] << 8) | buf1[22]);

		/* dig_H1 tek byte */
		cal.dig_H1 = buf1[25];

		/* dig_H2 signed */
		cal.dig_H2 = (int16_t)((buf2[1] << 8) | buf2[0]);

		/* dig_H3 tek byte */
		cal.dig_H3 = buf2[2];

		/* dig_H4 ve dig_H5 paketli gelir */
		cal.dig_H4 = (int16_t)((buf2[3] << 4) | (buf2[4] & 0x0F));
		cal.dig_H5 = (int16_t)((buf2[5] << 4) | (buf2[4] >> 4));

		/* dig_H6 signed 8-bit */
		cal.dig_H6 = (int8_t)buf2[6];

		/*
		 * 6. CTRL_HUM (Nem Oversampling)
		 * Nem için oversampling x8 seçilir
		 */
		uint8_t ctrl_hum;
		status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, &ctrl_hum, 1, BME280_I2C_TIMEOUT_MS);
		if (status != HAL_OK)
		{
			return status;
		}

		ctrl_hum = (ctrl_hum & 0xF8) | BME280_OSRS_X8;

		status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, &ctrl_hum, 1, BME280_I2C_TIMEOUT_MS);
		if (status != HAL_OK)
		{
			return status;
		}

		/*
		 * 7. CTRL_MEAS
		 * Sıcaklık ve basınç oversampling x1
		 * Mode = SLEEP
		 */
		uint8_t ctrl_meas;
		ctrl_meas = BME280_OSR_T_X1 | BME280_OSR_P_X1 | BME280_MODE_SLEEP;
		status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BME280_I2C_TIMEOUT_MS);
		if (status != HAL_OK)
		{
		    return status;
		}

		/*
		 * 8. CONFIG REGISTER (0xF5)
		 * IIR filter x4 seçilir
		 */
		uint8_t config = 0;
		config =  config | BME280_FILTER_4;
		status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_CONFIG, I2C_MEMADD_SIZE_8BIT, &config, 1, BME280_I2C_TIMEOUT_MS);
		if(status != HAL_OK)
		{
			return status;
		}


		return HAL_OK;
}

	//BME280 FORCED mode config ve  tek ölçüm tetikler
HAL_StatusTypeDef bme280_force_measure(void)
{
	HAL_StatusTypeDef status;
	uint8_t ctrl_meas;

	status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BME280_I2C_TIMEOUT_MS);
	if (status != HAL_OK)
	    {
	        return status;
	    }
	// Mode bitlerini temizle (bit1:0)
	ctrl_meas &= (uint8_t)~BME280_CTRL_MEAS_MODE_MASK;

	// Mode = FORCED yap
	ctrl_meas |= (uint8_t)BME280_MODE_FORCED;

	// Geri yaz -> ölçüm başlar
	status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1, BME280_I2C_TIMEOUT_MS);

	return status;

}

//BME280 şu anda ölçüm yapıyor mu kontrolü.
uint8_t bme280_is_measuring(void)
{
	HAL_StatusTypeDef status;
	uint8_t status_reg;
	status = HAL_I2C_Mem_Read(i2c1,BME280_SDO_LOW , BME280_STATUS, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, BME280_I2C_TIMEOUT_MS);
	if(status != HAL_OK)
	{
		return 1;
	}
	return (status_reg & BME280_STATUS_MEASURING_MASK);
}

/**
 * @brief  BME280'den ham (raw) basınç, sıcaklık ve nem verilerini okur.
 *
 * Sensörün 0xF7–0xFE adres aralığındaki ölçüm register'larından
 * toplam 8 byte okur ve datasheet'te tanımlandığı şekilde
 * 20-bit pressure, 20-bit temperature ve 16-bit humidity
 * ham ADC değerlerini oluşturur.
 *
 * @param  raw  Ham ölçüm değerlerinin yazılacağı yapı (bme280_raw_t)
 *              raw->adc_P : raw pressure (20-bit)
 *              raw->adc_T : raw temperature (20-bit)
 *              raw->adc_H : raw humidity (16-bit)
 */

HAL_StatusTypeDef bme280_read_raw(bme280_raw_t *raw)
{
	HAL_StatusTypeDef status;
	uint8_t buf[8];

	if (raw == NULL)
	{
		return HAL_ERROR;
	}

	/*
	 * 0xF7..0xFE arası 8 byte:
	 * [0]=press_msb, [1]=press_lsb, [2]=press_xlsb
	 * [3]=temp_msb,  [4]=temp_lsb,  [5]=temp_xlsb
	 * [6]=hum_msb,   [7]=hum_lsb
	 */
	status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_PRESS_MSB,I2C_MEMADD_SIZE_8BIT, buf, 8, BME280_I2C_TIMEOUT_MS);
	if (status != HAL_OK)
	{
		return status;
	}

	// 20-bit pressure: (MSB<<12) | (LSB<<4) | (XLSB>>4)
	raw->adc_P = (int32_t)((((uint32_t)buf[0]) << 12) | (((uint32_t)buf[1]) << 4) | (((uint32_t)buf[2]) >> 4));

	// 20-bit temperature: (MSB<<12) | (LSB<<4) | (XLSB>>4)
	raw->adc_T = (int32_t)((((uint32_t)buf[3]) << 12) | (((uint32_t)buf[4]) << 4) | (((uint32_t)buf[5]) >> 4));

	// 16-bit humidity: (MSB<<8) | LSB
	raw->adc_H = (int32_t)((((uint32_t)buf[6]) << 8) | ((uint32_t)buf[7]));

	return HAL_OK;
}

// Appendix A - 8.1 (double precision) Temperature compensation
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value

double bme280_compensate_T_double(int32_t adc_T)
{
	double var1, var2, T;

	var1 = (((double)adc_T) / 16384.0 - ((double)cal.dig_T1) / 1024.0) * ((double)cal.dig_T2);
	var2 = ((((double)adc_T) / 131072.0 - ((double)cal.dig_T1) / 8192.0) *
	        (((double)adc_T) / 131072.0 - ((double)cal.dig_T1) / 8192.0)) * ((double)cal.dig_T3);

	t_fine = (int32_t)(var1 + var2);

	T = (var1 + var2) / 5120.0;

	return T;  // °C
}

// Appendix A - 8.1 (double precision) Pressure compensation
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa

double bme280_compensate_P_double(int32_t adc_P)
{
    double var1, var2, p;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)cal.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)cal.dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)cal.dig_P4) * 65536.0);
    var1 = (((double)cal.dig_P3) * var1 * var1 / 524288.0 + ((double)cal.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)cal.dig_P1);

    if (var1 == 0.0)
    {
        return 0.0; // division by zero'yu önle
    }

    p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;

    var1 = ((double)cal.dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)cal.dig_P8) / 32768.0;
    p = p + (var1 + var2 + ((double)cal.dig_P7)) / 16.0;

    return p; // Pa
} // Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH

// Appendix A - 8.1 (double precision) Humidity compensation
double bme280_compensate_H_double(int32_t adc_H)
{
	double var_H;

	var_H = (((double)t_fine) - 76800.0);
	var_H = (adc_H - (((double)cal.dig_H4) * 64.0 + (((double)cal.dig_H5) / 16384.0) * var_H)) *
	        (((double)cal.dig_H2) / 65536.0 *
	         (1.0 + ((double)cal.dig_H6) / 67108864.0 * var_H *
	          (1.0 + ((double)cal.dig_H3) / 67108864.0 * var_H)));

	var_H = var_H * (1.0 - ((double)cal.dig_H1) * var_H / 524288.0);

	if (var_H > 100.0)
	{
		var_H = 100.0;
	}
	else if (var_H < 0.0)
	{
		var_H = 0.0;
	}

	return var_H; // %rH
}
