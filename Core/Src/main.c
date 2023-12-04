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
#include "state_machines.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HSP_SERVO_MIN_PULSE_WIDTH 500
#define HSP_SERVO_MAX_PULSE_WIDTH 2500
#define HSP_SERVO_PWM_PERIOD 20000
#define HSP_SERVO_MAX_DEG 180
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
struct Servo;
uint32_t Deg_To_CCR(uint8_t deg, const struct Servo *servo);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Servo {
  const char* pnid;
  volatile ServoState state;
  const TIM_HandleTypeDef *timer;
  volatile uint32_t *ccr;
};

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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  struct Servo servos[] = {
          {"FV-02", ServoState.INIT, &htim4, &TIM4->CCR1},
          {"FV-03", ServoState.INIT, &htim4, &TIM4->CCR2},
          {"FV-04", ServoState.INIT, &htim4, &TIM4->CCR3},
          {"FV-08", ServoState.INIT, &htim4, &TIM4->CCR4}
  };

  InhibitorState inhibitorState = InhibitorState.INIT;
  IgnitorState ignitorState = IgnitorState.INIT;


  // -- INIT -- //
  // close all servos, and set the gpi pins
  for (size_t i = 0; i < 4; i++ ) {
	  *servos[i].ccr = Deg_To_CCR(0, &servos[i]);
	  *servos[i].state = ServoState.READY;
  }

  // nothing for inhibitor right now
  inhibitorState = InhibitorState.INHIBIT;

  // nothing for ignitor right now
  ignitorState = IgnitorState.NOT_READY;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t deg = 0;

  uint8_t fireIgnitor = 0;
  uint8_t openServos = 0;

  uint8_t ignitorFired = 0;
  uint8_t servosOpened = 0;

  uint8_t abort = 0;

  while (1)
  {
	  // update states
	  if (abort) {
		  for (size_t i = 0; i < 4; i++ ) {
			 *servos[i].state = ServoState.NOT_READY;
		  }

		  inhibitorState = InhibitorState.INHIBIT;
		  ignitorState = IgnitorState.NOT_READY;
	  }

	  uint8_t allServosReady = 1;
	  for (size_t i = 0; i < 4; i++ ) {
		  allServosReady = (*servos[i].state == ServoState.READY) && allServosReady;
	  }

	  if (!allServosReady) {
		  inhibitorState = InhibitorState.INHIBIT;
	  }

	  // if any of the other data is bad:
	  // inhibitorState = InhibitorState.INHIBIT;




	  // -- actually do the state stuff -- //
	  // close all servos and turn everything off
	  if (abort) {
		  for (size_t i = 0; i < 4; i++) {
			  *servos[i].ccr = Deg_To_CCR(0, &servos[i]);
		  }

		  // potentially do inhibitor and ignitor stuff if needed here
	  }


	  // launch rocket
	  if (inhibitorState.CLEAR_FOR_LAUNCH && ignitorState.READY && allServosReady && fireIgnitor) {
		  // Fire_Ignitor();
		  ignitorFired = 1;
	  }

	  if (inhibitorState.CLEAR_FOR_LAUNCH && ignitorState.READY && allServosReady && fireIgnitor) {
		  // Fire_Ignitor();
		  servosOpened = 1;
	  }

	  if (!launched && !(inhibitorState == InhibitorState.INHIBIT)) {
		  if ()


		  for (size_t i = 0; i < 4; i++) {
			  if (ServoState.READY && ignitorState == IgnitorState.IGNITE) {

			  }
		  }




	  }
	  //


//    HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
//
//    for (size_t i = 0; i < 4; i++ ) {
//      *servos[i].ccr = Deg_To_CCR(deg, &servos[i]);
//    }
//
//    deg = deg == 0 ? 90 : 0;
//    HAL_Delay(1000);
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
uint32_t Deg_To_CCR(uint8_t deg, const struct Servo *servo) {
  uint32_t arr = servo->timer->Init.Period;
  double pulse_width = ((double)HSP_SERVO_MAX_PULSE_WIDTH-HSP_SERVO_MIN_PULSE_WIDTH)/HSP_SERVO_MAX_DEG * deg + HSP_SERVO_MIN_PULSE_WIDTH;
  return pulse_width*arr/HSP_SERVO_PWM_PERIOD;
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
