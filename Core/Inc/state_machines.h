/*
 * state_machines.h
 *
 *  Created on: Dec 3, 2023
 *      Author: brandonmarcus
 */

#ifndef INC_STATE_MACHINES_H_
#define INC_STATE_MACHINES_H_

// for servos
typedef struct {
	uint8_t INIT;

	uint8_t OPEN;

	uint8_t CLOSED;
}ServoState;

// for ignitor
typedef struct {
	uint8_t INIT;

	uint8_t READY;

	uint8_t IGNITE;

	uint8_t NOT_READY;
}IgnitorState;

// main struct determining whether or not to launch
typedef struct {
	uint8_t INIT;

	uint8_t INHIBIT;

	uint8_t CLEAR_FOR_LAUNCH;
}InhibitorState;



#endif /* INC_STATE_MACHINES_H_ */
