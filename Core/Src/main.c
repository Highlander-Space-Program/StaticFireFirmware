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
  const TIM_HandleTypeDef *timer;
  volatile uint32_t *ccr;
};

int start_tick_cnt = 0;

void Tick_NOS2 (uint8_t cmd, struct Servo *servo);
void Tick_NOS1 (uint8_t cmd, struct Servo *servo);
void Tick_N2 (uint8_t cmd, struct Servo *servo);
void Tick_ETOH (uint8_t cmd, struct Servo *servo);
void Tick_Start_Seq(uint8_t cmd, struct Servo servos[]);

enum COMMANDS {
  NOS_VALVE_2_TOGGLE,
  NOS_VALVE_1_TOGGLE,
  N2_VALVE_TOGGLE,
  ETOH_FLOW_VALVE_TOGGLE,
  START_SEQUENCE_1,
  FILL_SEQUENCE_1,
  FILL_SEQUENCE_2,
  FILL_SEQUENCE_3,
  CLOSE_ALL,
  IGNITE,
  ABORT
};

enum NOS2_STATE {NOS2_INIT, NOS2_CLOSED, NOS2_OPEN} nos2State = NOS2_INIT;
enum NOS1_STATE {NOS1_INIT, NOS1_CLOSED, NOS1_OPEN} nos1State = NOS1_INIT;
enum N2_STATE {N2_INIT, N2_CLOSED, N2_OPEN} n2State = N2_INIT;
enum ETOH_STATE {ETOH_INIT, ETOH_CLOSED, ETOH_OPEN} etohState = ETOH_INIT;
enum START_STATE {START_INIT, START_WAIT, START_OPEN_NOS, START_OPEN_ALL} startState = START_INIT;
uint8_t rx_buff[1];
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
          {"FV-02", &htim4, &TIM4->CCR1},
          {"FV-03", &htim4, &TIM4->CCR2},
          {"FV-04", &htim4, &TIM4->CCR3},
          {"FV-08", &htim4, &TIM4->CCR4}
  };

  HAL_UART_Receive_IT(&huart3, rx_buff, 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t deg = 0;
  while (1)
  {
    HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);

	HAL_Delay(1000);


    	//Tick_NOS2(rx_buff[i], &servos[3]);
    	//Tick_NOS1(rx_buff[i], &servos[2]);
    	//Tick_N2(rx_buff[i], &servos[0]);
    	Tick_ETOH(rx_buff[i], &servos[1]);
    	//Tick_Start_Seq(rx_buff[i], servos);
    	//Tick_Fill_1(rx_buff[i], &servos);
    	//Tick_Fill_2(rx_buff[i], &servos);
    	//Tick_Fill_3(rx_buff[i], &servos);
    	//Tick_Close(rx_buff[i], &servos);
    	//Tick_Ignite(rx_buff[i], &servos);
    	//Tick_Abort(rx_buff[i], &servos);

    HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);

    HAL_Delay(100);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart3, rx_buff, 10); //You need to toggle a breakpoint on this line!
}

void Tick_NOS2 (uint8_t cmd, struct Servo *servo) {
	//transitions
	switch(nos2State) {
		case NOS2_INIT:
		nos2State = NOS2_CLOSED;
		break;

		case NOS2_CLOSED:
		if (cmd == NOS_VALVE_2_TOGGLE) {
			nos2State = NOS2_OPEN;
		}
		else {
			nos2State = NOS2_CLOSED;
		}
		break;

		case NOS2_OPEN:
		if (cmd == NOS_VALVE_2_TOGGLE || cmd == CLOSE_ALL) {
			nos2State = NOS2_CLOSED;
		}
		else {
			nos2State = NOS2_OPEN;
		}
		break;
	}

	//actions
	switch(nos2State) {
		case NOS2_INIT:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case NOS2_CLOSED:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case NOS2_OPEN:
		*servo->ccr = Deg_To_CCR(90, servo);
		break;
	}
}

void Tick_NOS1 (uint8_t cmd, struct Servo *servo) {
	//transitions
	switch(nos1State) {
		case NOS1_INIT:
		nos1State = NOS1_CLOSED;
		break;

		case NOS1_CLOSED:
		if (cmd == NOS_VALVE_1_TOGGLE) {
			nos1State = NOS1_OPEN;
		}
		else {
			nos1State = NOS1_CLOSED;
		}
		break;

		case NOS1_OPEN:
		if (cmd == NOS_VALVE_1_TOGGLE || cmd == CLOSE_ALL) {
			nos1State = NOS1_CLOSED;
		}
		else {
			nos1State = NOS1_OPEN;
		}
		break;
	}

	//actions
	switch(nos1State) {
		case NOS1_INIT:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case NOS1_CLOSED:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case NOS1_OPEN:
		*servo->ccr = Deg_To_CCR(90, servo);
		break;
	}
}

void Tick_N2 (uint8_t cmd, struct Servo *servo) {
	//transitions
	switch(n2State) {
		case N2_INIT:
		n2State = N2_CLOSED;
		break;

		case N2_CLOSED:
		if (cmd == N2_VALVE_TOGGLE) {
			n2State = N2_OPEN;
		}
		else {
			n2State = N2_CLOSED;
		}
		break;

		case N2_OPEN:
		if (cmd == N2_VALVE_TOGGLE || cmd == CLOSE_ALL) {
			n2State = N2_CLOSED;
		}
		else {
			n2State = N2_OPEN;
		}
		break;
	}

	//actions
	switch(n2State) {
		case N2_INIT:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case N2_CLOSED:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case N2_OPEN:
		*servo->ccr = Deg_To_CCR(90, servo);
		break;
	}
}

void Tick_ETOH (uint8_t cmd, struct Servo *servo) {
	//transitions
	switch(etohState) {
		case ETOH_INIT:
		etohState = ETOH_CLOSED;
		break;

		case ETOH_CLOSED:
		if (cmd == ETOH_FLOW_VALVE_TOGGLE) {
			etohState = ETOH_OPEN;
		}
		else {
			etohState = ETOH_CLOSED;
		}
		break;

		case ETOH_OPEN:
		if (cmd == ETOH_FLOW_VALVE_TOGGLE || cmd == CLOSE_ALL) {
			etohState = ETOH_CLOSED;
		}
		else {
			etohState = ETOH_OPEN;
		}
		break;
	}

	//actions
	switch(etohState) {
		case ETOH_INIT:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case ETOH_CLOSED:
		*servo->ccr = Deg_To_CCR(0, servo);
		break;

		case ETOH_OPEN:
		*servo->ccr = Deg_To_CCR(90, servo);
		break;
	}
}

void Tick_Start_Seq(uint8_t cmd, struct Servo servos[]) {

	//transitions
	switch(startState) {
		case START_INIT:
		startState = START_WAIT;
		break;

		case START_WAIT:
		if (cmd == START_SEQUENCE_1) {
			startState = START_OPEN_NOS;
		}
		else {
			startState = START_WAIT;
		}
		break;

		case START_OPEN_NOS:
		if (cmd == CLOSE_ALL) {
			startState = START_INIT;
		}
		else if (start_tick_cnt > 5) {
			startState = START_OPEN_ALL;
		}
		else {
			++start_tick_cnt;
			startState = START_OPEN_NOS;
		}
		break;

		case START_OPEN_ALL:
		if (cmd == CLOSE_ALL || cmd == ABORT) {
			startState = START_INIT;
		}
		else {
			startState = START_OPEN_ALL;
		}
		break;
	}

	//actions
	switch(startState) {
		case START_INIT:
		for (int i = 0; i < 4; ++i) {
			const struct Servo *currServo = &servos[i];
			servos[i].ccr = Deg_To_CCR(0, currServo);
		}
		start_tick_cnt = 0;
		break;

		case START_WAIT:
		for (int i = 0; i < 4; ++i) {
			const struct Servo *currServo = &servos[i];
			servos[i].ccr = Deg_To_CCR(0, currServo);
		}
		break;

		case START_OPEN_NOS:
		servos[2].ccr = Deg_To_CCR(90, &servos[2]);
		servos[3].ccr = Deg_To_CCR(90, &servos[3]);
		break;

		case START_OPEN_ALL:
		for (int i = 0; i < 4; ++i) {
			servos[i].ccr = Deg_To_CCR(90, &servos[i]);
		}
		break;
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
