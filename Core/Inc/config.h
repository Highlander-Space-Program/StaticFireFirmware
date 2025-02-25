/*
 * config.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

//enum COMMANDS {
//  OPEN_NOS2 = 0,
//  CLOSE_NOS2 = 1,
//  OPEN_NOS1 = 2,
//  CLOSE_NOS1 = 3,
//  OPEN_N2 = 4,
//  CLOSE_N2 = 5,
//  OPEN_ETOH = 6,
//  CLOSE_ETOH = 7,
//  START_1 = 8,
//  FILL_1 = 9,
//  FILL_2 = 10,
//  FILL_3 = 11,
//  CLOSE_ALL = 12,
//  DECLOSE_ALL = 13,
//  ACTIVATE_IGNITER = 14,
//  DEACTIVATE_IGNITER = 15,
//  ABORT = 16,
//  ACTIVATE_SERVOS = 17,
//  DEACTIVATE_SERVOS = 18,
//  DEABORT = 19,
//  CHECK_STATE = 20,
//  DESTART = 21
//};

enum COMMANDS {
  OPEN_EO1 = 0,
  CLOSE_EO1 = 1,
  OPEN_NO6 = 2,
  CLOSE_NO6 = 3,
  OPEN_NO4 = 4,
  CLOSE_NO4 = 5,
  OPEN_NO3 = 6,
  CLOSE_NO3 = 7,
  START_1 = 8,
  OPEN_NO2 = 9,
  CLOSE_NO2 = 10,
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

struct Servo {
  const char* pnid;
  const TIM_HandleTypeDef *timer;
  volatile uint32_t *ccr;
};

bool etoh_just_opened = false;
int etoh_cnt = 0;
int ticks = 0;
bool isCloseAll = false;
bool isAborted = false;
bool isServoEnabled = false;
bool isStarted = false;
uint8_t ack = 0x00;

#define HSP_SERVO_MIN_PULSE_WIDTH 500
#define HSP_SERVO_MAX_PULSE_WIDTH 2500
#define HSP_SERVO_PWM_PERIOD 20000
#define HSP_SERVO_MAX_DEG 180
#define HSP_NO3_SERVO_MAX_DEG 270

#define EO1_OPENED_DEG 0
#define EO1_CLOSED_DEG 90

#define NO6_OPENED_DEG 0
#define NO6_CLOSED_DEG 90

#define NO4_OPENED_DEG 8
#define NO4_CLOSED_DEG 92

#define NO3_OPENED_DEG 0
#define NO3_CLOSED_DEG 90

#define NO2_OPENED_DEG 0
#define NO2_CLOSED_DEG 86

struct Servo servos[] = {
        {"NO6", &htim4, &TIM4->CCR1}, // used to be n2
        {"EO1", &htim4, &TIM4->CCR2}, // used to be etoh
        {"NO4", &htim4, &TIM4->CCR3}, // used to be nos1
        {"NO2", &htim3, &TIM3->CCR1}, // used to be nos2
		{"NO3", &htim1, &TIM1->CCR1}  // new one
};

enum IGNITER_STATE {IGNITER_INIT, IGNITER_DEACTIVATED, IGNITER_ACTIVATED} igniterState = IGNITER_INIT;
enum SERVO_STATE {SERVO_INIT, SERVO_CLOSED_OFF, SERVO_CLOSED_ON, SERVO_OPENED_ON, SERVO_OPENED_OFF};
enum SERVO_STATE eo1State = SERVO_INIT;
enum SERVO_STATE no6State = SERVO_INIT;
enum SERVO_STATE no4State = SERVO_INIT;
enum SERVO_STATE no3State = SERVO_INIT;
enum SERVO_STATE no2State = SERVO_INIT;
uint64_t no2_on_time = 0;
uint64_t no3_on_time = 0;
uint64_t no4_on_time = 0;
uint64_t no6_on_time = 0;
uint64_t eo1_on_time = 0;
uint8_t rx_buff[1];
uint8_t tx_buff[1];
#define SERVO_ON_TIME_MS 3000

#endif /* INC_CONFIG_H_ */
