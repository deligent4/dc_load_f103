/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdbool.h>
#include <stdio.h>
#include "ltc4151.h"
#include "mcp4725.h"
#include "PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A0		0		//LTC4151 A0 Pin HIGH->1, LOW->0
#define A1		0		//LTC4151 A1 Pin HIGH->1, LOW->0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t tick, prev_tick = 0;
uint16_t blink_delay = 500;
char stringValue[8];  // Adjust the buffer size as needed

LTC4151_t LTC4151;
uint16_t sense_resistor = 5;		//8 milli-ohms
uint16_t voltage = 0, voltage_snapshot = 0;
uint16_t current = 0, current_snapshot = 0;
uint8_t i, ret, i2c_add;
uint32_t desired_value, measured_value = 0;
float control_signal, prev_control_signal, control_signal_slew;
MCP4725 myMCP4725;
uint16_t next_voltage;

/* DAC Defines */
#define MCP47255_REF_VOLT 4096

/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  1.0f
#define PID_KD  0.0f

#define PID_TAU 0.02f

#define PID_LIM_MIN 	0.0f
#define PID_LIM_MAX  	4096.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data);
void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data);
void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data);
void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data);
float SlewRate_Limiter(float currentDACValue, float targetDACValue, float stepSize);
uint16_t setVoltageGradually(uint16_t currentValue, uint16_t finalValue, uint16_t stepSize);
uint16_t calculateNextVoltage(uint16_t currentValue, uint16_t finalValue, uint16_t stepSize);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc2);
  HAL_Delay(100);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  myMCP4725 = MCP4725_init(&hi2c1, MCP4725_ADDR, MCP47255_REF_VOLT);

  /* Initialise PID controller */
  PIDController pid = { PID_KP, PID_KI, PID_KD, PID_TAU, PID_LIM_MIN, PID_LIM_MAX,
		  	  	  	  	  PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S};
  PIDController_Init(&pid);

//  LTC4151_t_Init(hi2c2, &LTC4151, A0, A1, sense_resistor);

/*
*  Keep LTC4151 in shutdown state to avoid lockup of I2C bus
*/
//  HAL_GPIO_WritePin(LTC4151_SHDN_N_GPIO_Port, LTC4151_SHDN_N_Pin, SET);


//  for(i = 1; i < 127; i++){
//	  ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
//	  if (ret == HAL_OK) /* No ACK Received At That Address */
//	  {
//		  i2c_add = i;
//		  HAL_Delay(2000);
//	  }
//  	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  tick = HAL_GetTick();
//	  myOLED_int(10, 0, tick);
	  HAL_ADC_PollForConversion(&hadc1, 10);
	  desired_value = (HAL_ADC_GetValue(&hadc1) * 3300 / 4096);
//	  myOLED_int(10, 15, desired_value);

	  HAL_ADC_PollForConversion(&hadc2, 10);
	  measured_value = (HAL_ADC_GetValue(&hadc2) * 3300 / 4096);
//	  myOLED_int(10, 30, measured_value);

	  // Update PID controller with setpoint from potentiometer and measured voltage
	  control_signal = PIDController_Update(&pid, (float)desired_value, (float)measured_value);
//	  myOLED_float(10, 45, control_signal);

	  next_voltage = setVoltageGradually(desired_value, (uint16_t)control_signal, 10);

      MCP4725_Set_Voltage(&myMCP4725, next_voltage);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(tick - prev_tick >= blink_delay){
		  prev_tick = tick;
		  HAL_GPIO_TogglePin(LED_BLU_GPIO_Port, LED_BLU_Pin);
//		  ssd1306_Fill(Black);
	  }
//	  MCP4725_Set_Voltage(&myMCP4725, 1000);

//	  printf("Hello World,");
//	  printf("\r\n");
      // Update previous control signal
//	  prev_control_signal = control_signal;
	  HAL_Delay(200);
//	  ssd1306_UpdateScreen();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch){
	HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 10);
	return ch;
}
//"Project Properties > C/C++ Build > Settings > Tool Settings", or add manually "-u _printf_float" in linker flags.
void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data){

	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(data, Font_7x10, White);
}

void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data){
	char str_data[10];

	sprintf(str_data, "%f", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data){
	char str_data[10];

	sprintf(str_data, "%u", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data){
	char str_data[10];

	sprintf(str_data, "%d", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}


uint16_t setVoltageGradually(uint16_t currentValue, uint16_t finalValue, uint16_t stepSize) {
    static uint32_t lastStepTime = 0; // Record the time of the last step
    static uint32_t delayDuration = 10; // Delay duration in milliseconds

    uint32_t currentTime = HAL_GetTick(); // Get the current time

    // Check if it's time to take the next step
    if (currentTime - lastStepTime >= delayDuration) {
        // Update the current value
        if (currentValue < finalValue) {
            currentValue = (uint16_t)(currentValue + stepSize);
            if (currentValue > finalValue) {
                currentValue = finalValue;
            }
        } else {
            currentValue = (uint16_t)(currentValue - stepSize);
            if (currentValue < finalValue) {
                currentValue = finalValue;
            }
        }

        // Update the last step time
        lastStepTime = currentTime;
    }

    // Ensure the final value is returned precisely
    return currentValue;
}

uint16_t calculateNextVoltage(uint16_t currentValue, uint16_t finalValue, uint16_t stepSize) {
    static uint32_t lastStepTime = 0; // Record the time of the last step
    static uint32_t delayDuration = 10; // Delay duration in milliseconds

    uint32_t currentTime = HAL_GetTick(); // Get the current time

    // Check if it's time to take the next step
    if (currentTime - lastStepTime >= delayDuration) {
        // Update the current value
        if (currentValue < finalValue) {
            currentValue = (uint16_t)(currentValue + stepSize);
            if (currentValue > finalValue) {
                currentValue = finalValue;
            }
        } else {
            currentValue = (uint16_t)(currentValue - stepSize);
            if (currentValue < finalValue) {
                currentValue = finalValue;
            }
        }

        // Update the last step time
        lastStepTime = currentTime;
    }

    // Ensure the final value is returned precisely
    return currentValue;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
