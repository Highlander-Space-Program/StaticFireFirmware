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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "config.h"
#include "deg_to_ccr.h"
#include "eo1.h"
#include "no6.h"
#include "no4.h"
#include "no3.h"
#include "no2.h"
#include "create_ack.h"
#include "igniter.h"
#include "servo.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Tick_Components();
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
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */



  HAL_UART_Receive_IT(&huart3, rx_buff, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//activates servos
	if (rx_buff[0] == ACTIVATE_SERVOS) {
		Servo_Enable();
		isServoEnabled = true;
	}
	//deactivates servos
	else if (rx_buff[0] == DEACTIVATE_SERVOS) {
		Servo_Disable();
		isServoEnabled = false;
	}
	//if we get abort command disable servo signals and set flags
	if (rx_buff[0] == ABORT) {
		Servo_Disable();
		isServoEnabled = false;
		isAborted = true;
	}
	//remove abort flag if deabort
	else if (rx_buff[0] == DEABORT) {
		isAborted = false;
	}
	//set close all flag if we get close all cmd
	if (rx_buff[0] == CLOSE_ALL) {
		isCloseAll = true;
	}
	//remove close all flag if we get declose all cmd
	else if (rx_buff[0] == DECLOSE_ALL) {
		isCloseAll = false;
	}
	//set started flag if we get start cmd
	if (rx_buff[0] == START_1) {
		isStarted = true;
	}
	//removes started flag if we get destart cmd
	if (rx_buff[0] == DESTART) {
		isStarted = false;
	}

	Tick_Components();


    //creates and sends acknowledgement if a new command is received or 5 seconds have passed
    if (rx_buff[0] != 0xF0 || ticks >= 50) {
    	Create_Ack();
    	tx_buff[0] = ack;
    	HAL_UART_Transmit_IT(&huart3, tx_buff, 1);
    	ack = 0x00;
    	ticks = 0;
    }
//    HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
    rx_buff[0] = 0xF0;
    ++ticks;
    HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



//received uart byte gets put into rx_buff and interrupt re-enabled
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  Tick_Components();
  HAL_UART_Receive_IT(&huart3, rx_buff, 1);
}

void Tick_Components() {
	//ticks calls servo functions if servos have been enabled and abort isn't enabled
	if (isServoEnabled && !isAborted) {
		Tick_NO2(rx_buff[0], &servos[3]);
		Tick_NO4(rx_buff[0], &servos[2]);
		Tick_NO6(rx_buff[0], &servos[0]);
		Tick_EO1(rx_buff[0], &servos[1]);
		Tick_NO3(rx_buff[0], &servos[4]);
	}
	Tick_Igniter(rx_buff[0]);
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
