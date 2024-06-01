/*
 * nos2.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_NOS2_H_
#define INC_NOS2_H_

#include "config.h"
#include "deg_to_ccr.h"

//behavior for NOS2 servo
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

#endif /* INC_NOS2_H_ */
