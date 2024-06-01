/*
 * nos1.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_NOS1_H_
#define INC_NOS1_H_

#include "config.h"
#include "deg_to_ccr.h"

//behavior for NOS1 servo
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

#endif /* INC_NOS1_H_ */
