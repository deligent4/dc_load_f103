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
#include <stdbool.h>
#include <stdio.h>
#include "../../ssd1306_oled_lib/inc/ssd1306.h"
#include "../../ssd1306_oled_lib/inc/ssd1306_tests.h"
#include "../../ltc2944/ltc2944.h"
#include "../../ltc4151/ltc4151.h"
#include "../../mcp4725/mcp4725.h"
#include "../../pid/pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t tick, prev_tick = 0;
uint16_t blink_delay = 50;
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

ltc2944_configuration_t ltc2944_struct = {0};
extern ltc2944_data_t ltc2944_data;
uint16_t sec_prev = 0, seconds = 0;
uint8_t state;
uint8_t buf[2], status;

bool battery_detect = false;
bool is_ltc2944_config = false;


typedef enum {
	DEFAULT_OFF,
    CURRENT_500mA,
    CURRENT_1000mA,
    CURRENT_1500mA,
    CURRENT_2000mA,
    CURRENT_2500mA,
    CURRENT_3000mA,
    CURRENT_3500mA,
    CURRENT_4000mA,
    CURRENT_4500mA,
    CURRENT_5000mA,
    CURRENT_MAX // The number of available current settings
} LoadCurrent_t;

uint16_t dac_voltage[CURRENT_MAX] = {
		0,		// 0mA (OFF)
		251,  	// 500mA
		501,  	// 1000mA
		744,  	// 1500mA
		990,  	// 2000mA
		1233, 	// 2500mA
		1477, 	// 3000mA
		1720, 	// 3500mA
		1966, 	// 4000mA
		2214, 	// 4500mA
		2455  	// 5000mA
};

char* current_labels[CURRENT_MAX] = {
		"000mA"
		"500mA",
		"1000mA",
		"1500mA",
		"2000mA",
		"2500mA",
		"3000mA",
		"3500mA",
		"4000mA",
		"4500mA",
		"5000mA"
};

/* LOAD MAX MIN THRESHOLDs */
#define MAX_LOAD_VOLTAGE		30
#define MAX_LOAD_CURRENT		5


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
HAL_StatusTypeDef LTC2944_Device_Config(void);

typedef enum
{
	IDLE 		= 0,
	BATT_CONN	= 1,
	RUN			= 2,
}device_state_t;


// UART printf stuff
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(100);
  HAL_ADC_Start(&hadc1);

  myMCP4725 = MCP4725_init(&hi2c1, MCP4725_ADDR, MCP47255_REF_VOLT);
  LoadCurrent_t selected_current = DEFAULT_OFF;  // Default selection

  /* Initialise PID controller */
  PIDController pid = { PID_KP, PID_KI, PID_KD, PID_TAU, PID_LIM_MIN, PID_LIM_MAX,
		  	  	  	  	  PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S};
  PIDController_Init(&pid);

  // Print the basic format (main page)
  HAL_Delay(100);
  myOLED_char(1, 12, "Volt = ");
  myOLED_char(1, 24, "Curr = ");
  myOLED_char(1, 36, "Chg  = ");
  myOLED_char(1, 48, "Temp = ");
  ssd1306_UpdateScreen();
  HAL_Delay(100);

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
  selected_current = CURRENT_2000mA;
  MCP4725_Set_Voltage(&myMCP4725, dac_voltage[selected_current]);

  while (1)
  {
	  tick = HAL_GetTick();
	  myOLED_int(1, 2, tick);
//	  LTC2944_Init(ltc2944_struct);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//      MCP4725_Set_Voltage(&myMCP4725, dac_voltage[selected_current]);

	  HAL_ADC_PollForConversion(&hadc1, 10);
  	  if(HAL_ADC_GetValue(&hadc1) >= 200){
  		  battery_detect = true;
  	  }else if(HAL_ADC_GetValue(&hadc1) < 200){
  		  battery_detect = false;
  	  }
	  switch(state){
	  case IDLE:
		  if(battery_detect){
			  state = BATT_CONN;
		  }else{
			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
			  myOLED_char(50, 24, "        ");	// print empty spaces in curr
			  myOLED_char(50, 36, "       ");	// print empty spaces in chg
			  myOLED_char(50, 48, "  ");		// print empty spaces in temp
			  myOLED_int(50, 2, 0);
			  // Resets the seconds count every time battery is removed
			  if(seconds > 1){
				  seconds = 0;
			  }
		  }
		  break;

	  case BATT_CONN:
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET); 		// Turn on RED led for indication
		  LTC2944_Device_Config();
		  state = RUN;
		  break;

	  case RUN:
		  /*
		  * test timer for run condition
		  */
		  if(tick - sec_prev >= 1000){		// 1000ms = 1 sec
			  sec_prev = tick;
			  myOLED_int(50, 2, seconds++);
		  }
		  if(battery_detect){
			  status = LTC2944_Get_Battery_Data(&ltc2944_struct);
			  // print the battery values on oled screen
			  myOLED_float(50, 12, ltc2944_data.voltage);
			  myOLED_float(50, 24, ltc2944_data.current);
			  myOLED_float(50, 36, ltc2944_data.acc_charge);
			  myOLED_int(50, 48, ltc2944_data.temperature);
//			  if(status != HAL_OK){
//				  state = STUCK;
//				  break;
//			  }
		  }else{
			  state = IDLE;
		  }
		  break;

	  default:
	  }

	  if(tick - prev_tick >= blink_delay){
		  prev_tick = tick;
		  HAL_GPIO_TogglePin(LED_BLU_GPIO_Port, LED_BLU_Pin);
		  myOLED_int(75, 48, state);
		  myOLED_int(95, 48, status);
		  ssd1306_UpdateScreen();
	  }
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

HAL_StatusTypeDef LTC2944_Device_Config(void){
	ltc2944_struct.adc_mode 		=	Automatic_Mode;
	ltc2944_struct.alcc_mode 		= 	ALCC_Disable;
	ltc2944_struct.sense_resistor 	= 	5;
	ltc2944_struct.batt_capacity 	=	7000;
	ltc2944_struct.i2c_handle 		= 	hi2c2;
	ltc2944_struct.vth_max 			=	MAX_LOAD_VOLTAGE;
	ltc2944_struct.cth_max			= 	MAX_LOAD_CURRENT;
	return LTC2944_Init(ltc2944_struct);
}

void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data){

	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(data, Font_7x10, White);
}

void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data){
	char str_data[10];

	sprintf(str_data, "%.3f", data);
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
