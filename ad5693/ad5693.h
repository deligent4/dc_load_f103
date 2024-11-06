/*
 * ad569x.h
 *
 *  Created on: Dec 16, 2023
 *      Author: rahul
 */

#ifndef INC_AD569X_H_
#define INC_AD569X_H_

#include "i2c.h"
#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>


#define AD5693_ADDRESS			0x4C << 1			//0b01001100
// If A0 jumper is set high, use 0x4E

#define WRITE_INPUT_REG			0b00010000
#define UPDATE_DAC_REG			0b00100000
#define WRITE_DAC_N_INPUT_REG	0b00110000
#define WRITE_CONTROL_REG		0b01000000

#define DAC_RESET 				0x8000				// 1000000000000000
#define DAC_REF_ENABLE			0x1000				// 0001000000000000
#define DAC_SET_GAINx2			0x0800				// 0000100000000000
#define AD5693_VREF				2.5
#define I2C_HANDLE				&hi2c1


typedef enum{
	normal_mode 	= 0,	// Default mode
	power_down_1k	= 1,	// power down with 1k pull-down
	power_down_100k	= 2,	// power down with 100k pull-down
	tri_state		= 3		// power down with tri-state output
} ad5693_operating_mode_e;

typedef struct{
	bool ref_disable;
	bool gain_x2;
	ad5693_operating_mode_e mode;
}ad5693_configuration_t;

//extern ad5693_configuration_t ad5693;

HAL_StatusTypeDef AD5693_Init(ad5693_configuration_t ad5693);

HAL_StatusTypeDef AD5693_Set_Reg_Voltage(float value);
HAL_StatusTypeDef AD5693_Update_Reg(void);

HAL_StatusTypeDef AD5693_Set_Voltage(float value);
HAL_StatusTypeDef AD5693_Set_Voltage_Raw(uint16_t voltage);
HAL_StatusTypeDef AD5693_Reset(void);
uint16_t AD5693_Read_Input_Reg(void);
HAL_StatusTypeDef AD5693_Set_OP_Mode(ad5693_operating_mode_e mode);
#endif /* INC_AD569X_H_ */
