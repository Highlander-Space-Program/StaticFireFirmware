/*
 * n2.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_N2_H_
#define INC_N2_H_

#include "config.h"
#include "deg_to_ccr.h"

//behavior for N2 servo
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


#endif /* INC_N2_H_ */
