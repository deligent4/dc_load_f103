/*
 * ad569x.c
 *
 *  Created on: Dec 16, 2023
 *      Author: rahul
 *      reference https://github.com/adafruit/Adafruit_AD569x
 */

#include "ad5693.h"

HAL_StatusTypeDef AD5693_Reset(void){
	uint8_t command = WRITE_CONTROL_REG;		// Command Byte
	uint8_t highByte = 0b10000000;				// Data High, Reset bit High
	uint8_t lowByte = 0b00000000;				// Data Low

	// Combine the command and data into a single 3-byte buffer
	uint8_t buffer[3] = {command, highByte, lowByte};
	uint8_t status = HAL_I2C_Master_Transmit(I2C_HANDLE,
			AD5693_ADDRESS, buffer, 3, 10);
	return status;
}

HAL_StatusTypeDef AD5693_Init(ad5693_configuration_t ad5693){
	// Prepare the command byte
	uint8_t command = WRITE_CONTROL_REG;

	// Prepare the high and low data bytes
	uint16_t data = 0x0000;
	data |= ((uint16_t)ad5693.mode << 13); // Set D14 and D13 for the operating mode
	data |= ((uint16_t)ad5693.ref_disable << 12); // Set D12 for enable_ref
	data |= ((uint16_t)ad5693.gain_x2 << 11);      // Set D11 for the gain

	uint8_t highByte = (data >> 8) & 0xFF;
	uint8_t lowByte = data & 0xFF;

	// Combine the command and data into a single 3-byte buffer
	uint8_t buffer[3] = {command, highByte, lowByte};
	uint8_t status = HAL_I2C_Master_Transmit(I2C_HANDLE,
					AD5693_ADDRESS, buffer, 3, 10);
	return status;
}


HAL_StatusTypeDef AD5693_Set_Voltage(float voltage){

	if(voltage >= AD5693_VREF) voltage = AD5693_VREF;

	uint16_t data = voltage / AD5693_VREF * 65535.0;

	// Prepare the command byte
    uint8_t command = WRITE_DAC_N_INPUT_REG;

    // Prepare the high and low data bytes
    uint8_t highByte = (data >> 8) & 0xFF;
    uint8_t lowByte = data & 0xFF;

    // Combine the command and data into a single 3-byte buffer
    uint8_t buffer[3] = {command, highByte, lowByte};

    // Write the 3-byte buffer to the I2C device and return the result
	uint8_t status = HAL_I2C_Master_Transmit(I2C_HANDLE,
					AD5693_ADDRESS, buffer, 3, 10);
	return status;
}


HAL_StatusTypeDef AD5693_Set_OP_Mode(ad5693_operating_mode_e mode){
	// Prepare the command byte
	uint8_t command = WRITE_CONTROL_REG;
	// Prepare the high and low data bytes
	uint16_t data = 0x0000;
	switch(mode){
	case normal_mode:
		data |= ((uint16_t)normal_mode << 13); // Set D14 and D13 for the operating mode
		break;
	case power_down_1k:
		data |= ((uint16_t)power_down_1k << 13);
		break;
	case power_down_100k:
		data |= ((uint16_t)power_down_100k << 13);
		break;
	case tri_state:
		data |= ((uint16_t)tri_state << 13);
		break;
	default:
		break;
	}
	data |= ((uint16_t)0 << 12); 		// Set D12 for enable_ref
	data |= ((uint16_t)0 << 11);     	// Set D11 for the gain

	uint8_t highByte = (data >> 8) & 0xFF;
	uint8_t lowByte = data & 0xFF;

	// Combine the command and data into a single 3-byte buffer
	uint8_t buffer[3] = {command, highByte, lowByte};
	uint8_t status = HAL_I2C_Master_Transmit(I2C_HANDLE,
					AD5693_ADDRESS, buffer, 3, 10);
	return status;
}


HAL_StatusTypeDef AD5693_Set_Voltage_Raw(uint16_t voltage){
	uint16_t data = voltage;

	// Prepare the command byte
    uint8_t command = WRITE_DAC_N_INPUT_REG;

    // Prepare the high and low data bytes
    uint8_t highByte = (data >> 8) & 0xFF;
    uint8_t lowByte = data & 0xFF;

    // Combine the command and data into a single 3-byte buffer
    uint8_t buffer[3] = {command, highByte, lowByte};

    // Write the 3-byte buffer to the I2C device and return the result
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(I2C_HANDLE,
					AD5693_ADDRESS, buffer, 3, 10);
	return status;
}

/*
 |0100 0000 | 10000 000 | 0000 0000 |

 Command Byte - 0100 xxxx
 0000 - Do Nothing
 0001 - Write Input Register
 0010 - Update DAC Register
 0100 - Write Control Register

 Write Control Register
 D15 - Reset DAC
 D14-D13 - Operating Modes (power down)
 00 - Normal Mode
 01 - 1k output impedance
 10 - 100k output impedance
 11 - tri-state output
 D12 - Internal Reference
 D11 - Gain


 Data High Byte
 0000 0000

 Data Low Byte
 0000 0000
 */

