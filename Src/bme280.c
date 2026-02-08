/**
 * @brief  Initialize the BME280 sensor.
 *
 * This function performs a full initialization sequence:
 *  - Issues a soft reset
 *  - Verifies the chip ID
 *  - Waits until the sensor is ready
 *  - Reads factory calibration data
 *  - Configures oversampling, filter, and operating mode
 *
 * @return HAL status:
 *         - HAL_OK on success
 *         - HAL_ERROR or HAL_BUSY on failure
 */
HAL_StatusTypeDef bme280_init(void)
{

	HAL_StatusTypeDef status;

	// 1. SOFT RESET
	uint8_t pData = BME280_SOFT_RESET_CMD;
	status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_RESET,
	                          I2C_MEMADD_SIZE_8BIT, &pData, 1,
	                          BME280_I2C_TIMEOUT_MS);
	if(status != HAL_OK)
	{
		return status;
	}

	// 2. READ AND VERIFY CHIP ID
	uint8_t read;
	status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_ID,
	                         I2C_MEMADD_SIZE_8BIT, &read, 1,
	                         BME280_I2C_TIMEOUT_MS);
	if(status != HAL_OK)
	{
		return status;
	}
	if(read != BME280_CHIP_ID)
	{
		return HAL_ERROR;
	}

	// 3. WAIT UNTIL NVM COPY IS FINISHED
	uint8_t status_reg = 1;
	while (status_reg & 0x01)
	{
		HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_STATUS,
		                 I2C_MEMADD_SIZE_8BIT, &status_reg, 1,
		                 BME280_I2C_TIMEOUT_MS);
	}

	// 4. READ CALIBRATION DATA FROM SENSOR
	uint8_t buf1[BME280_CALIB_TP_LEN]; // 0x88 .. 0xA1
	uint8_t buf2[BME280_CALIB_H_LEN];  // 0xE1 .. 0xE7

	status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_CALIB_TP_START,
	                         I2C_MEMADD_SIZE_8BIT, buf1, sizeof(buf1),
	                         BME280_I2C_TIMEOUT_MS);
	if (status != HAL_OK)
	{
		return status;
	}

	status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_CALIB_H_START,
	                         I2C_MEMADD_SIZE_8BIT, buf2, sizeof(buf2),
	                         BME280_I2C_TIMEOUT_MS);
	if (status != HAL_OK)
	{
		return status;
	}

	/*
	 * 5. PARSE CALIBRATION COEFFICIENTS
	 * BME280 sends calibration data in little-endian format
	 * (LSB at lower address, MSB at higher address).
	 */

	/* dig_T1 (unsigned) */
	cal.dig_T1 = (uint16_t)((buf1[1] << 8) | buf1[0]);

	/* dig_T2 (signed) */
	cal.dig_T2 = (int16_t)((buf1[3] << 8) | buf1[2]);

	/* dig_T3 (signed) */
	cal.dig_T3 = (int16_t)((buf1[5] << 8) | buf1[4]);

	/* dig_P1 (unsigned) */
	cal.dig_P1 = (uint16_t)((buf1[7] << 8) | buf1[6]);

	/* dig_P2 .. dig_P9 (signed) */
	cal.dig_P2 = (int16_t)((buf1[9]  << 8) | buf1[8]);
	cal.dig_P3 = (int16_t)((buf1[11] << 8) | buf1[10]);
	cal.dig_P4 = (int16_t)((buf1[13] << 8) | buf1[12]);
	cal.dig_P5 = (int16_t)((buf1[15] << 8) | buf1[14]);
	cal.dig_P6 = (int16_t)((buf1[17] << 8) | buf1[16]);
	cal.dig_P7 = (int16_t)((buf1[19] << 8) | buf1[18]);
	cal.dig_P8 = (int16_t)((buf1[21] << 8) | buf1[20]);
	cal.dig_P9 = (int16_t)((buf1[23] << 8) | buf1[22]);

	/* dig_H1 (unsigned, single byte) */
	cal.dig_H1 = buf1[25];

	/* dig_H2 (signed) */
	cal.dig_H2 = (int16_t)((buf2[1] << 8) | buf2[0]);

	/* dig_H3 (unsigned, single byte) */
	cal.dig_H3 = buf2[2];

	/* dig_H4 and dig_H5 are packed across registers */
	cal.dig_H4 = (int16_t)((buf2[3] << 4) | (buf2[4] & 0x0F));
	cal.dig_H5 = (int16_t)((buf2[5] << 4) | (buf2[4] >> 4));

	/* dig_H6 (signed 8-bit) */
	cal.dig_H6 = (int8_t)buf2[6];

	/*
	 * 6. CONFIGURE CTRL_HUM REGISTER
	 * Humidity oversampling set to x8
	 */
	uint8_t ctrl_hum;
	status = HAL_I2C_Mem_Read(i2c1, BME280_SDO_LOW, BME280_CTRL_HUM,
	                         I2C_MEMADD_SIZE_8BIT, &ctrl_hum, 1,
	                         BME280_I2C_TIMEOUT_MS);
	if (status != HAL_OK)
	{
		return status;
	}

	ctrl_hum = (ctrl_hum & 0xF8) | BME280_OSRS_X8;

	status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_CTRL_HUM,
	                          I2C_MEMADD_SIZE_8BIT, &ctrl_hum, 1,
	                          BME280_I2C_TIMEOUT_MS);
	if (status != HAL_OK)
	{
		return status;
	}

	/*
	 * 7. CONFIGURE CTRL_MEAS REGISTER
	 * Temperature oversampling: x1
	 * Pressure oversampling   : x1
	 * Sensor mode             : SLEEP
	 */
	uint8_t ctrl_meas;
	ctrl_meas = BME280_OSR_T_X1 | BME280_OSR_P_X1 | BME280_MODE_SLEEP;
	status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_CTRL_MEAS,
	                          I2C_MEMADD_SIZE_8BIT, &ctrl_meas, 1,
	                          BME280_I2C_TIMEOUT_MS);
	if (status != HAL_OK)
	{
	    return status;
	}

	/*
	 * 8. CONFIGURE CONFIG REGISTER
	 * IIR filter coefficient set to x4
	 */
	uint8_t config = 0;
	config = config | BME280_FILTER_4;
	status = HAL_I2C_Mem_Write(i2c1, BME280_SDO_LOW, BME280_CONFIG,
	                          I2C_MEMADD_SIZE_8BIT, &config, 1,
	                          BME280_I2C_TIMEOUT_MS);
	if(status != HAL_OK)
	{
		return status;
	}

	return HAL_OK;
}
