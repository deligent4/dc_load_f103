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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "../../ssd1306_oled_lib/inc/ssd1306.h"
#include "../../ssd1306_oled_lib/inc/ssd1306_tests.h"
//#include "../../ltc2944/ltc2944.h"
//#include "../../ltc4151/ltc4151.h"
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
/*
 * MAXIMUM AND MINIMUM SETTINGS FOR LOAD
 */
#define MAX_CC_VALUE			5			// 5A maximum current
#define MIN_CC_VALUE			0.001		// 1mA minimum	current
#define MAX_CV_VALUE			30			// 30V maximum voltage
#define MIN_CV_VALUE			3			// 3V minimum voltage
#define MAX_CR_VALUE			10			// 10Ohm maximum resistance
#define MIN_CR_VALUE			0.1			// 100mOhm minimum resistance
#define MAX_CP_VALUE			99.999		// 99.999W maximum power CP is limited to 99.999Watt
#define MIN_CP_VALUE			1			// 1W minimum power

#define MAX_DIGITS 				6


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
uint16_t sense_resistor = 5;		//8 milli-ohms
uint32_t voltage = 0, charge = 0;
int16_t temperature = 0;
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
uint8_t temp_val[6] = {5};


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

// Define menu states
typedef enum {
	HOME_SCREEN,
	MODE_SELECTION,
	PARAMETER_SETTING,
	RETURN_TO_HOME
} Menu_State_e;

typedef struct {
	float voltage;
	float current;
	float power;
	float resistance;
} Param_Mode_t;


typedef struct {
	uint16_t set_voltage;
	uint16_t set_current;
	uint16_t set_power;
	uint16_t set_resistance;
} Set_Param_t;

Param_Mode_t param_mode = {0};
Set_Param_t set_param = {0};

Menu_State_e current_state = HOME_SCREEN;

uint32_t rotary_out, rot_pos, rot_cnt;

int cursor_position = 0;
int mode_index = -1;  // Store the index of mode setting
int mode_index_last = -1;
int last_cursor_position = -1;

float param_value = 0;
uint8_t digit_position = 0;
int last_rot_cnt = 0;
uint16_t sw_a_cnt = 0, sw_b_cnt = 0, sw_c_cnt = 0;
bool sw_rot_state = false,
		sw_a_state = false,
		sw_b_state = false,
		sw_c_state = false;
bool save_param = false;
bool adjusting_digit = false; // Flag to check if adjusting digit
volatile uint8_t digit_value = 0;
bool output_on_flag = false;
bool force_update;

uint16_t new_rot_pos, new_a_cnt ,new_b_cnt;
static uint16_t old_a_cnt = 0, old_b_cnt = 0, old_rot_pos = 0;


// 'ON', 140x81px
const unsigned char ON_BITMAP[] = { 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff,
		0xf8, 0xf8, 0x1f, 0x1f, 0x38, 0xf0, 0x0f, 0x0f, 0x38, 0xe1, 0xc7, 0x07,
		0x38, 0xe7, 0xe7, 0x07, 0x38, 0xc7, 0xe3, 0x03, 0x38, 0xc7, 0xe3, 0x23,
		0x38, 0xc7, 0xe3, 0x31, 0x38, 0xc7, 0xe3, 0x31, 0x38, 0xc7, 0xe3, 0x38,
		0x38, 0xc7, 0xe7, 0x38, 0x38, 0xe3, 0xc7, 0x3c, 0x38, 0xe0, 0x0f, 0x3e,
		0x38, 0xf8, 0x1f, 0x3e, 0x38, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff,
		0xf8 };

// 'OFF', 29x16px
const unsigned char OFF_BITMAP[] = { 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff,
		0xf8, 0xe0, 0x78, 0x08, 0x08, 0xc0, 0x38, 0x08, 0x08, 0x87, 0x18, 0xf8,
		0xf8, 0x8f, 0x18, 0xf8, 0xf8, 0x8f, 0x88, 0xf8, 0xf8, 0x8f, 0x88, 0x18,
		0x18, 0x8f, 0x88, 0x18, 0x18, 0x8f, 0x98, 0xf8, 0xf8, 0x8f, 0x18, 0xf8,
		0xf8, 0x80, 0x18, 0xf8, 0xf8, 0xc0, 0x38, 0xf8, 0xf8, 0xf0, 0xf9, 0xfd,
		0xf8, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xf8 };


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data);
void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data);
void myOLED_int16(uint16_t cursorX, uint16_t cursorY, uint16_t data);
void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data);
uint32_t Filter_Outliers(uint32_t* data, uint16_t length);

// Function Prototypes
void display_home_screen(bool force_update);
void display_mode_selection(bool force_update);
void display_parameter_setting(bool force_update);
void update_encoder_state();
void handle_button_press();
void update_display();
void update_digit_value(int direction);
void update_parameter_value(int direction); // function to update parameter value
void put_parameter_limit(void);				// put limit on parameter values
void Reset_Everything(void);
uint32_t concatenate(uint32_t x, uint32_t y);	// Concateate numbers


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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

  ssd1306_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(100);
  HAL_ADC_Start(&hadc1);

  printf("LTC2959 Begin\n\r");
//  while(HAL_I2C_IsDeviceReady(&LTC2959_I2C_PORT, LTC2959_I2C_ADDR, 100, 1000) != HAL_OK);	// wait for it to come alive
  LTC2959_Init(&ltc2959);
  HAL_Delay(100);

  AD5693_Reset();
  HAL_Delay(10);
  AD5693_Init(ad5693);
  HAL_Delay(10);

//  myOLED_char(1, 12, "Volt = ");
//  myOLED_char(1, 24, "Curr = ");
//  myOLED_char(1, 36, "Chg  = ");
//  myOLED_char(1, 48, "Temp = ");
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

  printf("RUNNING\n");
//  printf("%ld\n", concatenate(12,354));

  while (1)
  {
	  tick = HAL_GetTick();

	  update_display();
	  update_encoder_state();
	  handle_button_press();
	  set_param.set_current = param_mode.current * 1000;

//	  myOLED_int16(1, 2, tick);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_ADC_PollForConversion(&hadc1, 10);
  	  if(HAL_ADC_GetValue(&hadc1) >= 200){
  		  battery_detect = true;
  	  }else if(HAL_ADC_GetValue(&hadc1) < 200){
  		  battery_detect = false;
  	  }


      for (uint8_t i = 0; i < 10; i++) {
          sensor_data[i] = LTC2959_Get_Current();
      }

	  current = LTC2959_Get_Current();
  	  filter_current = Get_Current_Filtered(current);

/*
      if(tick - prev_control_delay >= control_delay){
		  voltage = LTC2959_Get_Voltage();
		  current = LTC2959_Get_Current();
		  charge = LTC2959_Get_Acc_Charge();
		  control_volt = Control_DAC_Output(500, current, battery_detect);
		  AD5693_Set_Voltage_Raw(control_volt);
		  prev_control_delay = tick;
	  }
*/


	  switch(state){
	  case IDLE:
		  if(battery_detect){
			  state = BATT_CONN;
		  }else{
			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
			  myOLED_char(50, 24, "        ");	// print empty spaces in curr
			  myOLED_char(50, 36, "       ");	// print empty spaces in chg
			  myOLED_char(50, 48, "  ");		// print empty spaces in temp
			  myOLED_int8(50, 2, 0);
			  // Resets the seconds count every time battery is removed
			  if(seconds > 1){
				  seconds = 0;
			  }
		  }
		  break;

	  case BATT_CONN:
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET); 		// Turn on RED led for indication
		  state = RUN;
		  break;

	  case RUN:
		  /*
		  * test timer for run condition
		  */
		  if(tick - sec_prev >= 1000){		// 1000ms = 1 sec
			  sec_prev = tick;
			  myOLED_int16(50, 2, seconds++);
		  }
		  if(battery_detect){
			  // print the battery values on oled screen
				  break;
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	rotary_out = __HAL_TIM_GET_COUNTER(htim);
	rot_cnt = rotary_out / 4;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
	case ROT_SW_Pin:
		sw_rot_state = true;
		if(current_state == PARAMETER_SETTING){
			save_param = true;	 	// raise the flag for saving param
		}else{
			save_param = false;
		}
		break;
	case A_SW_Pin:
		sw_a_state = true;
		sw_a_cnt++;
		break;
	case B_SW_Pin:
		sw_b_state = true;
		sw_b_cnt++;
		break;
	case C_SW_Pin:
		sw_c_state = true;
		sw_c_cnt++;
		break;
	default:
		break;
	}
}

void myOLED_char(uint16_t cursorX, uint16_t cursorY, char *data){

	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(data, Font_7x10, White);
}

void myOLED_char_big(uint16_t cursorX, uint16_t cursorY, char *data){

	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(data, Font_11x18, White);
}

void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data){
	char str_data[10];

	sprintf(str_data, "%.3f", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_float_big(uint16_t cursorX, uint16_t cursorY, float data){
	char str_data[10];

	sprintf(str_data, "%.3f", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_11x18, White);
}

void myOLED_int16(uint16_t cursorX, uint16_t cursorY, uint16_t data){
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


// Update Display
void update_display(){
	static Menu_State_e last_state = HOME_SCREEN;
	static bool first_update = true;
	force_update = (current_state != last_state) || first_update || !output_on_flag;

	if(force_update){
		last_state = current_state;
		last_cursor_position = -1; // Force full update on state change
		first_update = false;
	}

	// Handle cursor position updates separately
	bool cursor_changed = (cursor_position != last_cursor_position);

	switch(current_state){
	case HOME_SCREEN:
		display_home_screen(force_update || cursor_changed);
		break;
	case MODE_SELECTION:
		display_mode_selection(force_update || cursor_changed);
		break;
	case PARAMETER_SETTING:
		display_parameter_setting(force_update || cursor_changed);
		break;
	case RETURN_TO_HOME:
		current_state = HOME_SCREEN;
		break;
	default:
		break;
	}
	last_cursor_position = cursor_position;
}

// Display Home Screen
void display_home_screen(bool force_update){
	if(force_update || output_on_flag){
		ssd1306_Fill(Black);			// Clear the display before printing
		myOLED_char_big(0, 0, "V:");
		myOLED_float_big(21, 0, voltage);
		myOLED_char_big(0, 18, "C:");
		myOLED_float_big(21, 18, curr);
		myOLED_char_big(0, 36, "Q:");
		myOLED_float_big(21, 36, charge);
		myOLED_char(0, 54, "t:");
		myOLED_int16(15, 54, temperature);
		ssd1306_Line(78, 0, 78, 50, White);	// Draw line to separate the values and options
		myOLED_char(90, 0, "<SET>");			// SET MODE
		myOLED_char(90, 10, "<ON>");			// Turn ON LOAD
		myOLED_char(90, 20, "<RST>");			// Reset the LOAD

		// Show ON or OFF bitmap on display for LOAD status
		if(output_on_flag){
//			output_on_flag = false;									// Change the flag state
			myOLED_char(90, 10, "<OFF>");							// Print OFF in ON position if button is pressed
			ssd1306_DrawBitmap(90, 31, ON_BITMAP, 29, 16, White);	//	Draw ON bitmap
		}else{
			ssd1306_DrawBitmap(90, 31, OFF_BITMAP, 29, 16, White);	// Draw OFF bitmap
		}
	}

	// Update cursor only
	uint8_t scroll_num = 3;
	for(int i = 0; i < scroll_num; i++){
		if(i == cursor_position){
			myOLED_char(82, i * 10, ">");
		}else{
			myOLED_char(82, i * 10, " ");
		}
	}

	// Display the param value and mode if it is set
	if(((current_state == HOME_SCREEN) || save_param) || force_update){
		uint8_t x_mode = 50;
		uint8_t y_mode = 54;
		uint8_t x_val = 80;
		uint8_t y_val = 54;

		ssd1306_Line(45, y_mode, 45, 64, White);		// Draw line
		switch(mode_index){
		case 0:
			myOLED_char(x_mode, y_mode, "CC:");
			myOLED_float(x_val, y_val, param_mode.current);
			break;
		case 1:
			myOLED_char(x_mode, y_mode, "CV:");
			myOLED_float(x_val, y_val, param_mode.voltage);
			break;
		case 2:
			myOLED_char(x_mode, y_mode, "CP:");
			myOLED_float(x_val, y_val, param_mode.power);
			break;
		case 3:
			myOLED_char(x_mode, y_mode, "CR:");
			myOLED_float(x_val, y_val, param_mode.resistance);
			break;
		default:
			break;
		}
	}

	ssd1306_UpdateScreen();
}

// Display Mode Selection Screen
void display_mode_selection(bool force_update){
	const char *modes[] = { "CC", "CV", "CP", "CR" };
	if(force_update){
		ssd1306_Fill(Black); // Clear the screen
		for(int i = 0; i < 4; i++){
			myOLED_char(15, i * 10, (char*) modes[i]); // Print modes in column
		}
	}

	// Update cursor only
	uint8_t num_of_modes = 4;
	for(int i = 0; i < num_of_modes; i++){
		if(i == cursor_position){
			myOLED_char(0, i * 10, "->");
		}else{
			myOLED_char(0, i * 10, "  ");
		}
	}

	myOLED_char(5, 50, "<SELECT THE MODE>");
	ssd1306_UpdateScreen();
}

void display_parameter_setting(bool force_update){
	if(force_update){
		// Redraw entire screen if forced
		ssd1306_Fill(Black);
		myOLED_char(5, 0, "Set Value:");

		// Check the state and print the mode in parameter setting screen
		if(current_state == PARAMETER_SETTING){
			myOLED_char(5, 42, "MIN:");
			myOLED_char(5, 52, "MAX:");
			switch(mode_index){
			case 0:
				myOLED_char(70, 0, "CC");
				myOLED_char(70, 20, "mAmp");
				myOLED_float(35, 42, (float)MIN_CC_VALUE);
				myOLED_float(35, 52, (float)MAX_CC_VALUE);
				break;
			case 1:
				myOLED_char(70, 0, "CV");
				myOLED_char(70, 20, "Volt");
				myOLED_float(35, 42, (float)MIN_CV_VALUE);
				myOLED_float(35, 52, (float)MAX_CV_VALUE);
				break;
			case 2:
				myOLED_char(70, 0, "CP");
				myOLED_char(70, 20, "Watt");
				myOLED_float(35, 42, (float)MIN_CP_VALUE);
				myOLED_float(35, 52, (float)MAX_CP_VALUE);
				break;
			case 3:
				myOLED_char(70, 0, "CR");
				myOLED_char(70, 20, "Ohm");
				myOLED_float(35, 42, (float)MIN_CR_VALUE);
				myOLED_float(35, 52, (float)MAX_CR_VALUE);
				break;
			default:
				break;
			}
		}
	}

//	temp_val[0] =
//	float display_value;

//	for(uint8_t i = 0; i < sizeof(temp_val); i++){
//		for(uint8_t j = 1; j <= sizeof(temp_val); j++){
//			myOLED_int8(7*j, 20, temp_val[0]);
//		}
//	}

//	myOLED_int16(5, 20, display_value);
//	printf("display value: %d", display_value);
	// Display the value with proper formatting
	if(param_value >= 10.000){
		myOLED_float(0, 20, param_value);
	}else{
		myOLED_float(7, 20, param_value);
		myOLED_int16(0, 20, 0);  // Print "0" at the first location
	}

	// Clear previous cursor position by redrawing the entire line
	myOLED_char(0, 30, "         ");
	// Draw cursor under the digit
	uint8_t cursor_x;
	cursor_x = digit_position * 7;		// 7 pixels width per character
	myOLED_char(cursor_x , 30, "^");  	// Draw the cursor

	// Refresh the display after updating
	ssd1306_UpdateScreen();
}

// Update Encoder State
void update_encoder_state(){
	new_rot_pos = rot_cnt;

	if(new_rot_pos > old_rot_pos){
		if(current_state == PARAMETER_SETTING){
			if(adjusting_digit){
				update_parameter_value(1); // Increment digit
			}
		}else{
			cursor_position++;
		}
	}else if(new_rot_pos < old_rot_pos){
		if(current_state == PARAMETER_SETTING){
			if(adjusting_digit){
				update_parameter_value(-1); // Decrement digit
			}
		}else{
			cursor_position--;
		}
	}
	old_rot_pos = new_rot_pos;

	// Put limit on parameters and assign param_value
	// to specified mode
	put_parameter_limit();

	printf("param_value limit: %f\n\r\v", param_value);


	// putting limits
	if(cursor_position < 0)
		cursor_position = 0;
	if(current_state == HOME_SCREEN && cursor_position > 2)
		cursor_position = 2;
	if(current_state == MODE_SELECTION && cursor_position > 3)
		cursor_position = 3;
	if(current_state == PARAMETER_SETTING && digit_position > MAX_DIGITS)
		digit_position = 0;
}


void update_parameter_value(int direction) {
    // Convert the whole value to an integer, treating it as 00.000
    int full_value = (int)(param_value * 1000);

    // Extract individual digits
    int digits[5]; // [tens, ones, tenths, hundredths, thousandths]
    digits[0] = (full_value / 10000) % 10;     // tens
    digits[1] = (full_value / 1000) % 10;      // ones
    digits[2] = (full_value / 100) % 10;       // tenths
    digits[3] = (full_value / 10) % 10;        // hundredths
    digits[4] = full_value % 10;               // thousandths

    // Skip if we're at the decimal point position
    if (digit_position == 2) {
        return;
    }

    // Map cursor position to array index
    int digit_index = (digit_position < 2) ? digit_position : digit_position - 1;

    // Update the selected digit
    digits[digit_index] += direction;

    // Handle propagation and wrapping
    for (int i = digit_index; i >= 0; i--) {
        if (digits[i] > 9) {
            digits[i] = 0;
            if (i > 0) {
                digits[i-1]++;  // Carry over to the next digit
            }
        } else if (digits[i] < 0) {
            digits[i] = 9;
            if (i > 0) {
                digits[i-1]--;  // Borrow from the next digit
            }
        } else {
            break;  // Stop if no further propagation needed
        }
    }

    // Reconstruct the value
    full_value = digits[0] * 10000 +
                 digits[1] * 1000 +
                 digits[2] * 100 +
                 digits[3] * 10 +
                 digits[4];

    printf("full_value: %d\n\r", full_value);
    // Convert back to floating-point
    param_value = full_value / 1000.0;
}


// Put limit on parameter value
void put_parameter_limit(){
	// RESET the param_value if mode is changed
	if(mode_index_last != mode_index){
		param_value = 0;
		mode_index_last = mode_index;
	}

	// check the MIN and MAX value of each mode to set limits on the value.
	if(current_state == PARAMETER_SETTING){
		switch(mode_index){
		case 0:
			if(param_value >= MAX_CC_VALUE){
				param_value = MAX_CC_VALUE;
			}else if(param_value <= MIN_CC_VALUE){
				param_value = MIN_CC_VALUE;
			}
			if(save_param){
				param_mode.current = param_value;
				printf("current: %f\n\r", param_mode.current);

			}
			break;
		case 1:
			if(param_value >= MAX_CV_VALUE){
				param_value = MAX_CV_VALUE;
			}else if(param_value <= MIN_CV_VALUE){
				param_value = MIN_CV_VALUE;
			}
			if(save_param){
				param_mode.voltage = param_value;
				printf("voltage: %f\n\r", param_mode.voltage);
			}
			break;
		case 2:
			if(param_value >= MAX_CP_VALUE){
				param_value = MAX_CP_VALUE;
			}else if(param_value <= MIN_CP_VALUE){
				param_value = MIN_CP_VALUE;
			}
			if(save_param){
				param_mode.power = param_value;
				printf("power: %f\n\r", param_mode.power);}
			break;
		case 3:
			if(param_value >= MAX_CR_VALUE){
				param_value = MAX_CR_VALUE;
			}else if(param_value <= MIN_CR_VALUE){
				param_value = MIN_CR_VALUE;
			}
			if(save_param){
				param_mode.resistance= param_value;
				printf("resistance: %f\n\r", param_mode.resistance);
			}
			break;
		default:
			break;
		}
	}
}

void handle_button_press(){
	new_a_cnt = sw_a_cnt;
	new_b_cnt = sw_b_cnt;

	// Handle the EXIT button
	if(sw_c_state){
		if((current_state == MODE_SELECTION )||(current_state == PARAMETER_SETTING)){
			current_state = HOME_SCREEN;
			save_param = false;
			sw_c_state = false;
			Reset_Everything();
		}else if((current_state == HOME_SCREEN) && (cursor_position == 1)){
			Reset_Everything();
			sw_c_state = false;
		}
	}

	if(sw_rot_state){
		switch(current_state){
		case HOME_SCREEN:
			if(cursor_position == 0){
				current_state = MODE_SELECTION;		// GoTo MODE SELECTION PAGE
				output_on_flag = false;				// turn off when page is changed
			}else if((cursor_position == 1) && (!output_on_flag)){
				output_on_flag = true;
				// Handle "TURN ON" functionality
			}else if((cursor_position == 1) && output_on_flag){
				output_on_flag = false;
				force_update = true;
				// Handle "TURN ON" functionality
			}else if(cursor_position == 2){		// Reset everything
				Reset_Everything();
			}
			adjusting_digit = false;			// Disable adjusting in param setting
			break;
		case MODE_SELECTION:
			current_state = PARAMETER_SETTING;			// go to next state
			mode_index = cursor_position;
			cursor_position = 0;
			digit_position = 0;
			adjusting_digit = false;
			break;
		case PARAMETER_SETTING:
//			if(sw_rot_state){		// Return to home by saving param value
			current_state = RETURN_TO_HOME;
			digit_position = 0;
//			save_param = false;
//			}
			break;
		default:
			break;
		}
		sw_rot_state = false; // Reset button state
	}

	// Move cursor under the digits
	if(current_state == PARAMETER_SETTING) {
		adjusting_digit = true;
	}else adjusting_digit = false;

	if(new_b_cnt > old_b_cnt){
		digit_position++;
	}else if(new_a_cnt > old_a_cnt) {
		digit_position--;
	}
	old_b_cnt = new_b_cnt;
	old_a_cnt = new_a_cnt;
}

void Reset_Everything(void){
	current_state = HOME_SCREEN;	// not necessary to reset current_state
	cursor_position = 0;
	mode_index = -1;
	param_value = 0.0;
	output_on_flag = false;
	save_param = false;
	memset(&param_mode, 0, sizeof(param_mode));
	memset(&set_param, 0, sizeof(set_param));
}

uint32_t concatenate(uint32_t x, uint32_t y) {
    uint16_t pow = 1000;  		// For decimal places of 0.000
    if(y){
        while(y >= pow){
        pow *= 10;
        }
        return x * pow + y;
    }else{
        return x;
    }
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
