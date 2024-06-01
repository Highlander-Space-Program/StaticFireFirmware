/*
 * pwm_disable.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_PWM_DISABLE_H_
#define INC_PWM_DISABLE_H_

#include "config.h"
#include "tim.h"

//disables PWM signal to all servos
void PWM_Disable() {
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	  //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

#endif /* INC_PWM_DISABLE_H_ */
