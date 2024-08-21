/*
 * mcp4725.c
 *
 *  Created on: May 14, 2024
 *      Author: rahul
 */
#include "mcp4725.h"
#include "i2c.h"


MCP4725 MCP4725_init(I2C_HandleTypeDef* hi2c, MCP4725_I2C_ADDRESS addr, uint16_t refV)
{
	MCP4725 _MCP4725;
	_MCP4725._i2cAddress	= (uint16_t)(addr<<1);
	_MCP4725.hi2c			= hi2c;
	_MCP4725._refVoltage	= refV;
	return _MCP4725;
}


void MCP4725_Set_Voltage(MCP4725* _MCP4725, uint16_t value) {
	uint8_t data[2];

	if (_MCP4725->_refVoltage == 0) {
		// Avoid division by zero
		return;
	}

    // Calculate the DAC value based on the desired voltage
    uint16_t dac_value = (value * 4095) / _MCP4725->_refVoltage;
    if(dac_value > MCP4725_MAX_DAC_VALUE){
    		dac_value = MCP4725_MAX_DAC_VALUE;
    	}
    // Prepare the data to be sent to the MCP4725
    data[0] = dac_value >> 8; // MSB of DAC value
    data[1] = dac_value;        // LSB of DAC value
    // Send data to MCP4725
    HAL_I2C_Master_Transmit(_MCP4725->hi2c, _MCP4725->_i2cAddress, data, 2, HAL_MAX_DELAY);
}



void MCP4725_Set_Voltage_EEPROM(MCP4725* _MCP4725, uint16_t value){
	uint8_t data[3];

	if (_MCP4725->_refVoltage == 0) {
		// Avoid division by zero
		return;
	}

	uint16_t dac_value = (value * 4095) / _MCP4725->_refVoltage;
	if(dac_value > MCP4725_MAX_DAC_VALUE){
		dac_value = MCP4725_MAX_DAC_VALUE;
	}
	data[0] = MCP4725_CMD_WRITEDACEEPROM;
	data[1] = dac_value / 16;						// Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
	data[2] = (dac_value % 16) << 4;				// Lower data bits (D3.D2.D1.D0.x.x.x.x)

	HAL_I2C_Master_Transmit(_MCP4725->hi2c, _MCP4725->_i2cAddress, data, 3, HAL_MAX_DELAY);
}

