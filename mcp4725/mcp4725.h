/*
 * mcp4725.h
 *
 *  Created on: May 14, 2024
 *      Author: rahul
 */

#ifndef INC_MCP4725_H_
#define INC_MCP4725_H_

#include <stdint.h>
#include "i2c.h"

#define MCP4725_CMD_WRITEDAC (0x40)    ///< Writes data to the DAC
#define MCP4725_CMD_WRITEDACEEPROM (0x60) ///< Writes data to the DAC and the EEPROM (persisting the assigned
#define MCP4725_MAX_DAC_VALUE		4095

typedef enum {
    MCP4725_ADDR = 0x62					//MCP4725 I2C ADDRESS
}MCP4725_I2C_ADDRESS;


// MCP4725 class
typedef struct
{
	// Privates:
	I2C_HandleTypeDef* 		hi2c;
	MCP4725_I2C_ADDRESS 		_i2cAddress;
	uint16_t             	_refVoltage;
} MCP4725;


extern MCP4725 _MCP4725;



MCP4725 MCP4725_init(I2C_HandleTypeDef* hi2c, MCP4725_I2C_ADDRESS addr, uint16_t refV);
void MCP4725_Set_Voltage(MCP4725* _MCP4725, uint16_t value);
void MCP4725_Set_Voltage_EEPROM(MCP4725* _MCP4725, uint16_t value);

#endif /* INC_MCP4725_H_ */
