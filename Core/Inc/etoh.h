/*
 * etoh.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_ETOH_H_
#define INC_ETOH_H_

#include "config.h"
#include "deg_to_ccr.h"
#include "create_ack.h"

//behavior for etoh servo
void Tick_ETOH (uint8_t cmd, struct Servo *servo) {
	//transitions
	switch(etohState) {
		case ETOH_INIT:
		etohState = ETOH_CLOSED;
		break;

		case ETOH_CLOSED:
		if ((cmd == START_1 && !isCloseAll) && !isAborted) {
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

#endif /* INC_ETOH_H_ */
