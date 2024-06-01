/*
 * config.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

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

struct Servo servos[] = {
        {"FV-02", &htim4, &TIM4->CCR1},
        {"FV-03", &htim4, &TIM4->CCR2},
        {"FV-04", &htim4, &TIM4->CCR3},
        {"FV-08", &htim3, &TIM3->CCR1}
};

enum NOS2_STATE {NOS2_INIT, NOS2_CLOSED, NOS2_OPENED} nos2State = NOS2_INIT;
enum NOS1_STATE {NOS1_INIT, NOS1_CLOSED, NOS1_OPENED} nos1State = NOS1_INIT;
enum N2_STATE {N2_INIT, N2_CLOSED, N2_OPENED} n2State = N2_INIT;
enum ETOH_STATE {ETOH_INIT, ETOH_CLOSED, ETOH_WAIT, ETOH_OPENED} etohState = ETOH_INIT;
enum IGNITER_STATE {IGNITER_INIT, IGNITER_DEACTIVATED, IGNITER_ACTIVATED} igniterState = IGNITER_INIT;
uint8_t rx_buff[1];
uint8_t tx_buff[1];

#endif /* INC_CONFIG_H_ */
