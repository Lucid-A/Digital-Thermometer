/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define A 0
#define B 1
#define C 2
#define D 3
#define E 4
#define F 5
#define G 6
#define DP 7
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t LED[13][7] = {
{1,1,1,1,1,1,0},//0
{0,1,1,0,0,0,0},//1
{1,1,0,1,1,0,1},//2
{1,1,1,1,0,0,1},//3
{0,1,1,0,0,1,1},//4
{1,0,1,1,0,1,1},//5
{1,0,1,1,1,1,1},//6
{1,1,1,0,0,0,0},//7
{1,1,1,1,1,1,1},//8
{1,1,1,1,0,1,1},//9
{1,0,0,1,1,1,0},//C
{0,0,0,0,0,0,1},//-
{0,0,0,0,0,0,0} //Blank
};

uint8_t hold = 0;
uint8_t unit = 0;
float temperature;
const uint16_t PIN[8] = {
	A_Pin,  //A
	B_Pin,	//B
	C_Pin,	//C
	D_Pin,	//D
	E_Pin,	//E
	F_Pin,	//F
	G_Pin,	//G
	DP_Pin};	//DP
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int show_ch(uint8_t pos, char ch, uint8_t dot);
void LED_on(uint8_t pos);
void LED_off(uint8_t pos);
void all_off();
void show4bit(char c1, char c2, char c3, char c4, uint16_t time, uint8_t dot);
void show_id();
void temperature_display();
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	show_id();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		temperature_display();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int show_ch(uint8_t pos, char ch, uint8_t dot){
	uint8_t num = 16;
	uint8_t i;

	if(ch >= '0' && ch <= '9'){
		num = ch - '0';
	}else{
		switch(ch){
			case 'C':num = 10;break;
			case '-':num = 11;break;
			default: num = 12;break;
		}
	}
	
	for(i = 0;i < 7;++i){
		HAL_GPIO_WritePin(GPIOB, PIN[i], LED[num][i]);
	}
	HAL_GPIO_WritePin(GPIOB, PIN[DP], dot);
	LED_on(pos);
	HAL_Delay(3);
	LED_off(pos);
	return num;
}
void LED_on(uint8_t pos){
	switch(pos){
		case 1:HAL_GPIO_WritePin(CC1_GPIO_Port, CC1_Pin, GPIO_PIN_SET);break;
		case 2:HAL_GPIO_WritePin(CC2_GPIO_Port, CC2_Pin, GPIO_PIN_SET);break;
		case 3:HAL_GPIO_WritePin(CC3_GPIO_Port, CC3_Pin, GPIO_PIN_SET);break;
		case 4:HAL_GPIO_WritePin(CC4_GPIO_Port, CC4_Pin, GPIO_PIN_SET);break;
		default:break;
	}
}
void LED_off(uint8_t pos){
	all_off();
	switch(pos){
		case 1:HAL_GPIO_WritePin(CC1_GPIO_Port, CC1_Pin, GPIO_PIN_RESET);break;
		case 2:HAL_GPIO_WritePin(CC2_GPIO_Port, CC2_Pin, GPIO_PIN_RESET);break;
		case 3:HAL_GPIO_WritePin(CC3_GPIO_Port, CC3_Pin, GPIO_PIN_RESET);break;
		case 4:HAL_GPIO_WritePin(CC4_GPIO_Port, CC4_Pin, GPIO_PIN_RESET);break;
		default:break;
	}
}
void all_off(){
	uint8_t i;
	for(i = 0; i < 8; ++i){
		HAL_GPIO_WritePin(GPIOB,PIN[i],GPIO_PIN_RESET);
	}
}

void show4bit(char c1, char c2, char c3, char c4, uint16_t time, uint8_t dot){
	uint16_t i;
	if (time < 12) time = 12;
	for(i = 0;i < time/12; ++i){
		show_ch(1,c1,(dot & 0x40)>>6);
		show_ch(2,c2,(dot & 0x10)>>4);
		show_ch(3,c3,(dot & 0x04)>>2);
		show_ch(4,c4, dot & 0x01);
	}
}

void show4num(uint16_t num, uint16_t time, uint8_t dot){
	uint8_t bit[4]={0};
	bit[0] = (uint8_t)(num / 1000);
	bit[1] = (uint8_t)((num / 100) % 10);
	bit[2] = (uint8_t)((num % 100) / 10);
	bit[3] = (uint8_t)(num % 10);
	show4bit(bit[0]+'0',bit[1]+'0',bit[2]+'0',bit[3]+'0',100,dot);
}

void show_id(){
	uint16_t time = 100;
	show4bit(' ', ' ', ' ', '1',time,0);
	show4bit(' ', ' ', '1', '9',time,0);
	show4bit(' ', '1', '9', '5',time,0);
	show4bit('1', '9', '5', '0',time,0);
	show4bit('9', '5', '0', '0',time,0);
	show4bit('5', '0', '0', '5',time,0);
	show4bit('0', '0', '5', '3',time,0);
	show4bit('0', '5', '3', ' ',time,0);
	show4bit('5', '3', ' ', ' ',time,0);
	show4bit('3', ' ', ' ', ' ',time,0);
	show4bit(' ', ' ', ' ', ' ',400,0);	
}

void temperature_display(){
	float t; 
	float tc = temperature;
	float tf = tc * 1.8f + 32.0f;
	float tk = tc + 273.15f;
	do{
		uint16_t x;
		switch(unit){
			case 0: t = tc; break;
			case 1: t = tf; break;
			case 2: t = tk; break;
		}
		uint8_t dot;
		uint16_t time = 100;
		if (t > 100.0f){
			x = (uint16_t)(t * 10.0f + 0.5);
			dot = 0x04;
		}else if (t > 10.0f){
			x = (uint16_t)(t * 100.0f + 0.5f);
			dot = 0x10;
		}else if (t > 10.0f){
			x = (uint16_t)(t * 100.0f + 0.5f);
			dot = 0x40;
		}else{
			show4bit('-', '-', '-', '-',time,0);
			return;
		}
		show4num(x,time,dot);
		switch(unit){
				case 0:
					HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED_K_GPIO_Port, LED_K_Pin, GPIO_PIN_RESET);
					break;
				case 1:
					HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED_K_GPIO_Port, LED_K_Pin, GPIO_PIN_RESET);
					break;
				case 2:
					HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED_K_GPIO_Port, LED_K_Pin, GPIO_PIN_SET);
					break;
		}
	}while(hold);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
  {
    case KEY_HOLD_Pin:
			hold = !hold;
			if(hold){
				HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);
			}
			break;
    case KEY_SWITCH_Pin: 
			unit = (unit + 1) % 3;
			break;
		default:  break;
  }
	 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
