/*
 * LTC4151_t.h
 *
 *  Created on: Nov 17, 2023
 *      Author: rahul
 */

#ifndef INC_LTC4151_H_
#define INC_LTC4151_H_

#include "stm32f1xx_hal.h"
#include "i2c.h"

#define LTC4151_I2C_HANDLE			hi2c2


// Register directions
#define REG_SENSE_H 				0x00 		// (A) SENSE (8 MSBs)
#define REG_SENSE_L 				0x01 		// (B) SENSE (4 LSBs)
#define REG_VIN_H 					0x02   		// (C) VIN (8 MSBs)
#define REG_VIN_L 					0x03   		// (D) VIN (4 LSBs)
#define REG_ADIN_H 					0x04 		// (E) ADIN (8 MSBs)
#define REG_ADIN_L 					0x05  		// (F) ADIN (4 MSBs)
#define REG_CTRL 					0x06    	// (G) CONTROL


#define CTRL_BIT_SNAPSHOT_ENABLE 			0x7
#define CTRL_BIT_ADC_CHN_SNAPSHOT_MODE 		0x5
#define CTRL_BIT_TEST_MODE_ENABLE 			0x4
#define CTRL_BIT_PAGE_RW_ENABLE 			0x3
#define CTRL_BIT_STUCK_BUS_TIMER_ENABLE 	0x2


#define SNAPSHOT_CHANNEL_SENSE 				0x0
#define SNAPSHOT_CHANNEL_VIN 				0x1
#define SNAPSHOT_CHANNEL_ADIN 				0x2


#define FACTORx1000							(uint16_t)1000
#define VOLTAGE_CORRECTION					(uint16_t)200		//voltage correction value in mV

typedef struct {
	volatile uint8_t 	L; 					// Low
	volatile uint8_t 	H; 					// High
	volatile uint8_t 	NC; 				// Pin Floating
	volatile uint16_t 	_i2c_add;			//I2C Address
	uint16_t	 		_sense_resistor;	//Sense resistor value in milli-ohms
	I2C_HandleTypeDef 	_i2c_handle;		//I2C handle
}LTC4151_t;


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
void LTC4151_t_Init(I2C_HandleTypeDef i2c_handle, LTC4151_t *LTC4151_t, uint8_t A0, uint8_t A1, uint16_t sense_resistor);


/**
  * @brief  Measures voltage between S+ and S- terminals
  * 		and calculates current passing through the sense resistor.
  * @param	LTC4151_t Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of current passing through the load resistor (Rs).
  */
uint16_t Get_Load_Current(LTC4151_t *LTC4151_t);


/**
  * @brief  Measures voltage in VIN terminal.
  * @param	LTC4151_t Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VIN.
  */
uint16_t Get_Input_Voltage(LTC4151_t *LTC4151_t);


/**
  * @brief  Measures voltage in VADIN terminal.
  * @param	LTC4151_t Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VADIN.
  */
uint16_t Get_ADCIn_Voltage(LTC4151_t *LTC4151_t);


/**
  * @brief  Takes a snapshot of voltage between S+ and S- terminals
  * 		and calculates current passing through the sense resistor.
  * @param	LTC4151_t Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of current passing through the load resistor (Rs).
  */
uint16_t Get_Snapshot_Load_Current(LTC4151_t *LTC4151_t);


/**
  * @brief  Takes a snapshot of voltage in VIN terminal.
  * @param	LTC4151_t Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VIN.
  */
uint16_t Get_Snapshot_Input_Voltage(LTC4151_t *LTC4151_t);


/**
  * @brief  Takes a snapshot of voltage in VADIN terminal.
  * @param	LTC4151_t Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VADIN.
  */
uint16_t Get_Snapshot_ADCIn_Voltage(LTC4151_t *LTC4151_t);


/**
  * @brief  Disables Snapshot Mode and sets ADC channel to default.
  * @param	ltc4151 Pointer to a LTC4151_t structure that contains the
  *         configuration of the LTC4151_t IC.
  * @retval None.
  */
void Disable_Snapshot_Mode(LTC4151_t *ltc4151, uint8_t ctrl_reg);


#endif /* INC_LTC4151_H_ */
