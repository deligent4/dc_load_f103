/*
 * ltc4151.c
 *
 *  Created on: Nov 17, 2023
 *      Author: rahul
 */


#include "ltc4151.h"
//uint8_t buf[2];
//uint8_t h, l;
//uint16_t result;

/**
  * @brief  Initializes LTC4151 sensor.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @param  A0 Value of Address Pin 0 (L (0) = Tie to GND; H (1) = Tie High; NC (2) = Open;).
  * @param  A1 Value of Address Pin 1 (L (0) = Tie to GND; H (1) = Tie High; NC (2) = Open;).
  * @param 	sense_resistor- value of sense resistor in milli-ohms
  * @retval None.
  */
void LTC4151_t_Init(I2C_HandleTypeDef i2c_handle, LTC4151_t *ltc4151,
		uint8_t A0, uint8_t A1, uint16_t sense_resistor)
{
	ltc4151->_i2c_handle = i2c_handle;

	ltc4151->L 	= (uint8_t)0;
	ltc4151->H 	= (uint8_t)1;
	ltc4151->NC = (uint8_t)2;
	// Set the I2C address depending on ADR0 and ADR1 pins
	if (A0 == ltc4151->L && A1 == ltc4151->H) ltc4151->_i2c_add = 0b1100111;
	else if (A0 == ltc4151->H  && A1 == ltc4151->NC) ltc4151->_i2c_add = 0b1101000;
	else if (A0 == ltc4151->H  && A1 == ltc4151->H ) ltc4151->_i2c_add = 0b1101001;
	else if (A0 == ltc4151->NC && A1 == ltc4151->NC) ltc4151->_i2c_add = 0b1101010;
	else if (A0 == ltc4151->L  && A1 == ltc4151->NC) ltc4151->_i2c_add = 0b1101011;
	else if (A0 == ltc4151->H  && A1 == ltc4151->L ) ltc4151->_i2c_add = 0b1101100;
	else if (A0 == ltc4151->NC && A1 == ltc4151->H ) ltc4151->_i2c_add = 0b1101101;
	else if (A0 == ltc4151->NC && A1 == ltc4151->L ) ltc4151->_i2c_add = 0b1101110;
	else if (A0 == ltc4151->L  && A1 == ltc4151->L ) ltc4151->_i2c_add = 0b1101111;
	ltc4151->_i2c_add = ltc4151->_i2c_add << 1;
	ltc4151->_sense_resistor = sense_resistor;
}


/**
  * @brief  Reads a register.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @param  reg Register to read.
  * @param  numOfBytes Number of bytes to read.
  * @retval Contents of numOfBytes consecutive registers starting at register reg.
  */
static uint16_t Read_ADC(LTC4151_t *ltc4151, uint16_t reg, uint16_t numOfBytes)
{
	uint8_t buf[2];
	uint8_t h, l;
	uint16_t result;
	uint8_t status;

	status = HAL_I2C_Mem_Read(&(ltc4151->_i2c_handle), ltc4151->_i2c_add,
					reg, 1, buf, numOfBytes, 1000);
	if(status == HAL_OK)
	{
		if (numOfBytes == 1)
		{
			result = buf[0];
		} else if (numOfBytes == 2)
		{
			h = buf[0];
			l = buf[1];
			result = h << 4 | l >> 4;
		}
	}else
	{
		return 0;		//return 0, if there is no device connected
	}

	return result;
}


/**
  * @brief  Reads a register one time when previous conversion is complete.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @param  reg Register to read.
  * @retval Contents of 2 consecutive registers starting at register reg.
  */
static uint16_t Read_ADC_Snapshot(LTC4151_t *ltc4151, uint16_t reg)
{
	uint8_t buf[2];
	uint8_t h, l;
	uint16_t result;
	uint8_t status;

	status = HAL_I2C_Mem_Read(&(ltc4151->_i2c_handle), ltc4151->_i2c_add,
						reg, 1, buf, 2, HAL_MAX_DELAY);
	if(status == HAL_OK)
	{
		h = buf[0];
		l = buf[1];
		result = h << 4 | l >> 4;
	}else
	{
		return 0;		//return 0, if there is no device connected
	}
	return result;
}


/**
  * @brief  Writes to CONTROL Register G (06h).
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @param  ctrl_reg Value to write to control register.
  * @retval None.
  */
static void Set_Control_Register(LTC4151_t *ltc4151, uint8_t ctrl_reg)
{
	uint8_t buf[1];
	buf[0] = ctrl_reg;

	HAL_I2C_Mem_Write(&(ltc4151->_i2c_handle), ltc4151->_i2c_add,
						REG_CTRL, 1, buf, 1, HAL_MAX_DELAY);
}


/**
  * @brief  Gets the content of the control register.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval Value of CONTROL Register G (06h).
  */
static uint8_t Get_Control_Register(LTC4151_t *ltc4151)
{
	return (uint8_t)(Read_ADC(ltc4151, REG_CTRL, 1));
}


/**
  * @brief  Measures voltage between S+ and S- terminals
  * 		and calculates current passing through the sense resistor.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval Value of current passing through the load resistor(Rs) in milli-Amps.
  */
uint16_t Get_Load_Current(LTC4151_t *ltc4151)
{
	uint16_t scale_factor = (81.92 * FACTORx1000) / 4096;

 	return Read_ADC(ltc4151, REG_SENSE_H, 2) * scale_factor / ltc4151->_sense_resistor;
}


/**
  * @brief  Measures voltage in VIN terminal.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval Value of voltage on pin SENSE+ in milli-Volts.
  */
uint16_t Get_Input_Voltage(LTC4151_t *ltc4151)
{
	return (Read_ADC(ltc4151, REG_VIN_H, 2) * (102.4 * FACTORx1000) / 4096) - VOLTAGE_CORRECTION;
}


/**
  * @brief  Measures voltage in VADIN terminal.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval Value of voltage in VADIN.
  */
uint16_t Get_ADCIn_Voltage(LTC4151_t *ltc4151)
{
	return Read_ADC(ltc4151, REG_ADIN_H, 2) * (2.048 * FACTORx1000) / 4096;
}


/**
  * @brief  Takes a snapshot of voltage between S+ and S- terminals
  * 		and calculates current passing through the sense resistor.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval Value of current passing through the load resistor (Rs).
  */
uint16_t Get_Snapshot_Load_Current(LTC4151_t *ltc4151)
{
	uint8_t ctrl_reg = Get_Control_Register(ltc4151);
	Disable_Snapshot_Mode(ltc4151, ctrl_reg);

	ctrl_reg |= SNAPSHOT_CHANNEL_SENSE << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
	Set_Control_Register(ltc4151, ctrl_reg);

	uint16_t scale_factor = (81.92 * FACTORx1000) / 4096;
	return Read_ADC_Snapshot(ltc4151, REG_SENSE_H) * scale_factor / ltc4151->_sense_resistor;
}


/**
  * @brief  Takes a snapshot of voltage in VIN terminal.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval Value of voltage in VIN.
  */
uint16_t Get_Snapshot_Input_Voltage(LTC4151_t *ltc4151)
{
	uint8_t ctrlReg = Get_Control_Register(ltc4151);
	Disable_Snapshot_Mode(ltc4151, ctrlReg);

	ctrlReg |= SNAPSHOT_CHANNEL_VIN << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
	Set_Control_Register(ltc4151, ctrlReg);

	return (Read_ADC_Snapshot(ltc4151, REG_VIN_H) * (102.4 * FACTORx1000) / 4096) - VOLTAGE_CORRECTION;
}


/**
  * @brief  Takes a snapshot of voltage in VADIN terminal.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval Value of voltage in VADIN.
  */
uint16_t Get_Snapshot_ADCIn_Voltage(LTC4151_t *ltc4151)
{
	uint8_t ctrlReg = Get_Control_Register(ltc4151);
	Disable_Snapshot_Mode(ltc4151, ctrlReg);

	ctrlReg |= SNAPSHOT_CHANNEL_ADIN << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
	Set_Control_Register(ltc4151, ctrlReg);

	return Read_ADC_Snapshot(ltc4151, REG_ADIN_H) * (2.048 * FACTORx1000) / 4096;
}


/**
  * @brief  Disables Snapshot Mode and sets ADC channel to default.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval None.
  */
void Disable_Snapshot_Mode(LTC4151_t *ltc4151, uint8_t ctrl_reg)
{
	if ((ctrl_reg & (1 << CTRL_BIT_SNAPSHOT_ENABLE)) > 0)
	{
		//reset snapshot bit
		ctrl_reg = ctrl_reg & ~(1 << CTRL_BIT_SNAPSHOT_ENABLE);

		//reset adc channel to SENSE
		ctrl_reg = ctrl_reg & ~(1 << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE);
		ctrl_reg = ctrl_reg & ~(1 << (CTRL_BIT_ADC_CHN_SNAPSHOT_MODE + 1));

		Set_Control_Register(ltc4151, ctrl_reg);
	}
}
