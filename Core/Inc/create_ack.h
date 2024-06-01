/*
 * create_ack.h
 *
 *  Created on: Jun 1, 2024
 *      Author: zan
 */

#ifndef INC_CREATE_ACK_H_
#define INC_CREATE_ACK_H_

#include "config.h"

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

#endif /* INC_CREATE_ACK_H_ */
