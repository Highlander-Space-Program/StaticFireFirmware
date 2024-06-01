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

bool etoh_just_opened = false;
int etoh_cnt = 0;
bool isCloseAll = false;
bool isAborted = false;
bool isServoEnabled = false;
bool isStarted = false;
uint8_t ack = 0x00;

void Tick_NOS2 (uint8_t cmd, struct Servo *servo);
void Tick_NOS1 (uint8_t cmd, struct Servo *servo);
void Tick_N2 (uint8_t cmd, struct Servo *servo);
void Tick_ETOH (uint8_t cmd, struct Servo *servo);
void Tick_Igniter(uint8_t cmd);
void PWM_Enable();
void PWM_Disable();
uint8_t Create_Ack();

enum COMMANDS {
  OPEN_NOS2 = 0,
  CLOSE_NOS2 = 1,
  OPEN_NOS1 = 2,
  CLOSE_NOS1 = 3,
  OPEN_N2 = 4,
  CLOSE_N2 = 5,
  OPEN_ETOH = 6,
  CLOSE_ETOH = 7,
  START_1 = 8,
  FILL_1 = 9,
  FILL_2 = 10,
  FILL_3 = 11,
  CLOSE_ALL = 12,
  DECLOSE_ALL = 13,
  ACTIVATE_IGNITER = 14,
  DEACTIVATE_IGNITER = 15,
  ABORT = 16,
  ACTIVATE_SERVOS = 17,
  DEACTIVATE_SERVOS = 18,
  DEABORT = 19,
  CHECK_STATE = 20,
  DESTART = 21
};

enum NOS2_STATE {NOS2_INIT, NOS2_CLOSED, NOS2_OPENED} nos2State = NOS2_INIT;
enum NOS1_STATE {NOS1_INIT, NOS1_CLOSED, NOS1_OPENED} nos1State = NOS1_INIT;
enum N2_STATE {N2_INIT, N2_CLOSED, N2_OPENED} n2State = N2_INIT;
enum ETOH_STATE {ETOH_INIT, ETOH_CLOSED, ETOH_WAIT, ETOH_OPENED} etohState = ETOH_INIT;
enum IGNITER_STATE {IGNITER_INIT, IGNITER_DEACTIVATED, IGNITER_ACTIVATED} igniterState = IGNITER_INIT;
uint8_t rx_buff[1];
uint8_t tx_buff[1];
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
  /* USER CODE BEGIN 2 */


  struct Servo servos[] = {
          {"FV-02", &htim4, &TIM4->CCR1},
          {"FV-03", &htim4, &TIM4->CCR2},
          {"FV-04", &htim4, &TIM4->CCR3},
          {"FV-08", &htim3, &TIM3->CCR1}
  };
  HAL_UART_Receive_IT(&huart3, rx_buff, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (rx_buff[0] == ACTIVATE_SERVOS) {
		PWM_Enable();
		isServoEnabled = true;
	}
	else if (rx_buff[0] == DEACTIVATE_SERVOS) {
		PWM_Disable();
		isServoEnabled = false;
	}

	if (rx_buff[0] == ABORT) {
		PWM_Disable();
		isServoEnabled = false;
		isAborted = true;
	}
	else if (rx_buff[0] == DEABORT) {
		isAborted = false;
	}

	if (rx_buff[0] == CLOSE_ALL) {
		isCloseAll = true;
	}
	else if (rx_buff[0] == DECLOSE_ALL) {
		isCloseAll = false;
	}

	if (rx_buff[0] == START_1) {
		isStarted = true;
	}
	if (rx_buff[0] == DESTART) {
		isStarted = false;
	}

	if (isServoEnabled && !isAborted) {
		Tick_NOS2(rx_buff[0], &servos[3]);
		Tick_NOS1(rx_buff[0], &servos[2]);
		Tick_N2(rx_buff[0], &servos[0]);
		Tick_ETOH(rx_buff[0], &servos[1]);
	}
    Tick_Igniter(rx_buff[0]);



    if (rx_buff[0] != 0xF0) {
    	Create_Ack();
    	tx_buff[0] = ack;
    	HAL_UART_Transmit_IT(&huart3, tx_buff, 1);
    	ack = 0x00;
    }
    HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
    rx_buff[0] = 0xF0;

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
  HAL_UART_Receive_IT(&huart3, rx_buff, 1);
}

void Tick_NOS2 (uint8_t cmd, struct Servo *servo) {
	//transitions
	switch(nos2State) {
		case NOS2_INIT:
		nos2State = NOS2_CLOSED;
		break;

		case NOS2_CLOSED:
		if (((cmd == OPEN_NOS2 && !isCloseAll) && !isAborted)
			|| ((cmd == START_1 && !isCloseAll) && !isAborted)) {
			nos2State = NOS2_OPENED;
		}
		else {
			nos2State = NOS2_CLOSED;
		}
		break;

		case NOS2_OPENED:
		if ((cmd == CLOSE_NOS2 || cmd == CLOSE_ALL) && !isAborted && !isStarted) {
			nos2State = NOS2_CLOSED;
		}
		else {
			nos2State = NOS2_OPENED;
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

		case NOS2_OPENED:
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
		if (((cmd == OPEN_NOS1 && !isCloseAll) && !isAborted && !isStarted)) {
			nos1State = NOS1_OPENED;
		}
		else {
			nos1State = NOS1_CLOSED;
		}
		break;

		case NOS1_OPENED:
		if ((cmd == CLOSE_NOS1 || cmd == CLOSE_ALL || cmd == START_1) && !isAborted) {
			nos1State = NOS1_CLOSED;
		}
		else {
			nos1State = NOS1_OPENED;
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

		case NOS1_OPENED:
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
		if (((cmd == OPEN_N2 && !isCloseAll) && !isAborted)
			|| ((cmd == START_1 && !isCloseAll) && !isAborted)) {
			n2State = N2_OPENED;
		}
		else {
			n2State = N2_CLOSED;
		}
		break;

		case N2_OPENED:
		if ((cmd == CLOSE_N2 || cmd == CLOSE_ALL) && !isAborted && !isStarted) {
			n2State = N2_CLOSED;
		}
		else {
			n2State = N2_OPENED;
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

		case N2_OPENED:
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
		if (cmd == START_1) {
			etohState = ETOH_WAIT;
		}
		else if ((cmd == OPEN_ETOH && !isCloseAll) && !isAborted && !isStarted) {
			etohState = ETOH_OPENED;
		}
		else {
			etohState = ETOH_CLOSED;
		}
		break;

		case ETOH_WAIT:
		if (cmd == ABORT) {
			etohState = ETOH_CLOSED;
		}
		else if (etoh_cnt > 4) {
			etoh_just_opened = true;
			etohState = ETOH_OPENED;
		}
		break;

		case ETOH_OPENED:
		if (etoh_just_opened == true) {
			etoh_just_opened = false;
			ack = 0x00;
			Create_Ack();
			tx_buff[0] = ack;
			HAL_UART_Transmit_IT(&huart3, tx_buff, 1);
		}
		if ((cmd == CLOSE_ETOH || cmd == CLOSE_ALL) && !isAborted && !isStarted) {
			etohState = ETOH_CLOSED;
		}
		else {
			etohState = ETOH_OPENED;
		}
		break;
	}

	//actions
	switch(etohState) {
		case ETOH_INIT:
		*servo->ccr = Deg_To_CCR(0, servo);
		etoh_cnt = 0;
		break;

		case ETOH_CLOSED:
		*servo->ccr = Deg_To_CCR(0, servo);
		etoh_cnt = 0;
		break;

		case ETOH_WAIT:
		*servo->ccr = Deg_To_CCR(0, servo);
		++etoh_cnt;
		break;

		case ETOH_OPENED:
		*servo->ccr = Deg_To_CCR(90, servo);
		break;
	}
}

void Tick_Igniter(uint8_t cmd) {
	//transitions
	switch(igniterState) {
		case IGNITER_INIT:
		igniterState = IGNITER_DEACTIVATED;
		break;

		case IGNITER_DEACTIVATED:
		if (cmd == ACTIVATE_IGNITER && (!isCloseAll && !isAborted)) {
			igniterState = IGNITER_ACTIVATED;
		}
		else {
			igniterState = IGNITER_DEACTIVATED;
		}
		break;

		case IGNITER_ACTIVATED:
		if (((cmd == DEACTIVATE_IGNITER || cmd == CLOSE_ALL || isCloseAll)  && !isStarted) || cmd == ABORT) {
			igniterState = IGNITER_DEACTIVATED;
		}
		else {
			igniterState = IGNITER_ACTIVATED;
		}
		break;
	}

	//actions
	switch(igniterState) {
		case IGNITER_INIT:
		HAL_GPIO_WritePin(GPIOC, IGNITER_Pin, GPIO_PIN_RESET);
		break;

		case IGNITER_DEACTIVATED:
		HAL_GPIO_WritePin(GPIOC, IGNITER_Pin, GPIO_PIN_RESET);
		break;

		case IGNITER_ACTIVATED:
		HAL_GPIO_WritePin(GPIOC, IGNITER_Pin, GPIO_PIN_SET);
		break;
	}
}

void PWM_Enable() {
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void PWM_Disable() {
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	  //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

uint8_t Create_Ack() {
	if (nos2State == NOS2_OPENED) {
		++ack;
	}
	ack = ack << 1;
	if (nos1State == NOS1_OPENED) {
		++ack;
	}
	ack = ack << 1;
	if (n2State == N2_OPENED) {
		++ack;
	}
	ack = ack << 1;
	if (etohState == ETOH_OPENED) {
		++ack;
	}
	ack = ack << 1;
	if (igniterState == IGNITER_ACTIVATED) {
		++ack;
	}
	return ack;
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
