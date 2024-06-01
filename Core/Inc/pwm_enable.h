/*
 * pwm_enable.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_PWM_ENABLE_H_
#define INC_PWM_ENABLE_H_

#include "config.h"
#include "tim.h"

void PWM_Enable() {
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

#endif /* INC_PWM_ENABLE_H_ */
