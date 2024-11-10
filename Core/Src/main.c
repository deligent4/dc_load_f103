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
//#include "../../ltc2944/ltc2944.h"
#include "../../ltc4151/ltc4151.h"
#include "../../mcp4725/mcp4725.h"
#include "../../pid/pid.h"
#include "../../ltc2959/ltc2959.h"
#include "../../ad5693/ad5693.h"
#include "../../filter/filter.h"


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
int32_t curr, current = 0, filter_current = 0;
LTC4151_t LTC4151;
uint16_t sense_resistor = 5;		//8 milli-ohms
uint32_t voltage = 0, charge = 0;
uint8_t i, ret, i2c_add;
uint32_t desired_value, measured_value = 0;
MCP4725 myMCP4725;
uint16_t next_voltage;

uint16_t sec_prev = 0, seconds = 0;
uint8_t state;
uint8_t buf[2], status;
uint16_t control_volt;
uint32_t prev_print_delay = 0, print_delay = 100;
uint32_t prev_control_delay = 0, control_delay = 10;
bool battery_detect = false ;
uint32_t sensor_data[10];


LTC2959_Config_t ltc2959 = {
    .ADC_mode		= 	CTRL_ADC_MODE_CONT_I,
    .GPIO_config 	= 	CTRL_GPIO_CONFIG_ANALOG_INPUT_1560mV,
    .voltage_input 	= 	CTRL_CONFIG_VOLTAGE_INPUT_SENSEN,
    .CC_deadband 	= 	CC_CONFIG_DEADBAND_20,
};

ad5693_configuration_t ad5693 = {
		.gain_x2 		= false,
		.ref_disable	= false,
		.mode 		= normal_mode,
};



/*
 * MAXIMUM AND MINIMUM SETTINGS FOR LOAD
 */
#define MAX_CC_VALUE			5.0			// 5A maximum current
#define MIN_CC_VALUE			0.001		// 1mA minimum	current
#define MAX_CV_VALUE			30			// 30V maximum voltage
#define MIN_CV_VALUE			3			// 3V minimum voltage
#define MAX_CR_VALUE			10			// 10Ohm maximum resistance
#define MIN_CR_VALUE			0.1			// 100mOhm minimum resistance
#define MAX_CP_VALUE			99.999		// 99.999W maximum power CP is limited to 99.999Watt
#define MIN_CP_VALUE			1			// 1W minimum power

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data);
void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data);
void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data);
void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data);
uint32_t Filter_Outliers(uint32_t* data, uint16_t length);

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

  printf("LTC2959 Begin\n\r");
  while(HAL_I2C_IsDeviceReady(&LTC2959_I2C_PORT, LTC2959_I2C_ADDR, 100, 1000) != HAL_OK);	// wait for it to come alive
  LTC2959_Init(&ltc2959);
  HAL_Delay(1000);

  AD5693_Reset();
  HAL_Delay(10);
  AD5693_Init(ad5693);
  HAL_Delay(10);




  myOLED_char(1, 12, "Volt = ");
  myOLED_char(1, 24, "Curr = ");
  myOLED_char(1, 36, "Chg  = ");
  myOLED_char(1, 48, "Temp = ");
  ssd1306_UpdateScreen();
  HAL_Delay(100);

//  for(i = 1; i < 127; i++){
//	  ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i), 3, 5);
//	  if (ret == HAL_OK) /* No ACK Received At That Address */
//	  {
//		  printf("%x \n\r", i);
//		  i2c_add = i;
//		  HAL_Delay(1000);
//	  }
//  	}	  HAL_Delay(20000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  printf("RUNNING");

  while (1)
  {
	  tick = HAL_GetTick();
//	  myOLED_int(1, 2, tick);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_ADC_PollForConversion(&hadc1, 10);
  	  if(HAL_ADC_GetValue(&hadc1) >= 200){
  		  battery_detect = true;
  	  }else if(HAL_ADC_GetValue(&hadc1) < 200){
  		  battery_detect = false;
  	  }


//      for (uint8_t i = 0; i < 10; i++) {
//          sensor_data[i] = LTC2959_Get_Current();
//      }

//	  current = LTC2959_Get_Current();
//  	  filter_current = Get_Current_Filtered(current);

      if(tick - prev_control_delay >= control_delay){
		  voltage = LTC2959_Get_Voltage();
		  current = LTC2959_Get_Current();
		  charge = LTC2959_Get_Acc_Charge();
//		  control_volt = Control_DAC_Output(500, current, battery_detect);
		  AD5693_Set_Voltage_Raw(control_volt);
		  prev_control_delay = tick;
	  }



	  if(tick - prev_print_delay >= print_delay){
//		  printf("LTC2959_Voltage = %.4f\n\r", voltage);
//		  printf("LTC2959_current = %ld, Filter_current = %ld\n\r\v", current, filter_current);
//		  printf("%ld, %ld, %ld \n\r", voltage, current, filter_current);
//		  printf("LTC2959_charge = %.4f\n\r\v", charge);
		  printf("LTC2959_current = %ld\n\r", current);
		  prev_print_delay = tick;
	  }

//	  switch(state){
//	  case IDLE:
//		  if(battery_detect){
//			  state = BATT_CONN;
//		  }else{
//			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
//			  myOLED_char(50, 24, "        ");	// print empty spaces in curr
//			  myOLED_char(50, 36, "       ");	// print empty spaces in chg
//			  myOLED_char(50, 48, "  ");		// print empty spaces in temp
//			  myOLED_int(50, 2, 0);
//			  // Resets the seconds count every time battery is removed
//			  if(seconds > 1){
//				  seconds = 0;
//			  }
//		  }
//		  break;
//
//	  case BATT_CONN:
//		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET); 		// Turn on RED led for indication
//		  state = RUN;
//		  break;
//
//	  case RUN:
//		  /*
//		  * test timer for run condition
//		  */
//		  if(tick - sec_prev >= 1000){		// 1000ms = 1 sec
//			  sec_prev = tick;
//			  myOLED_int(50, 2, seconds++);
//		  }
//		  if(battery_detect){
//			  // print the battery values on oled screen
//				  break;
////			  }
//		  }else{
//			  state = IDLE;
//		  }
//		  break;
//
//	  default:
//	  }

	  if(tick - prev_tick >= blink_delay){
		  prev_tick = tick;
		  HAL_GPIO_TogglePin(LED_BLU_GPIO_Port, LED_BLU_Pin);
//		  myOLED_int(75, 48, state);
//		  myOLED_int(95, 48, status);
//		  ssd1306_UpdateScreen();
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


uint32_t Filter_Outliers(uint32_t* data, uint16_t length) {
    uint32_t sum = 0;
    uint32_t max_value = 0;
    uint32_t min_value = UINT32_MAX;

    // Calculate the sum, max, and min of the data
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
        if (data[i] > max_value) {
            max_value = data[i];
        }
        if (data[i] < min_value) {
            min_value = data[i];
        }
    }

    // Calculate the mean without floating-point math
    uint32_t mean = sum / length;

    // Calculate the standard deviation without floating-point math
    uint32_t variance = 0;
    for (uint16_t i = 0; i < length; i++) {
        int32_t diff = (int32_t)data[i] - (int32_t)mean;
        variance += diff * diff;
    }
    variance /= length;
    uint32_t std_dev = (uint32_t)sqrt((double)variance);

    // Filter out outliers
    uint32_t filtered_value = 0;
    uint16_t outlier_count = 0;
    for (uint16_t i = 0; i < length; i++) {
        if (data[i] > THRESHOLD || abs((int32_t)data[i] - (int32_t)mean) > STD_THRESHOLD * (int32_t)std_dev) {
            filtered_value += mean;
            outlier_count++;
        } else {
            filtered_value += data[i];
        }
    }

    // Return the filtered value, or the original value if no outliers were detected
    return outlier_count > 0 ? filtered_value / (length - outlier_count) : LTC2959_Get_Current();
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
