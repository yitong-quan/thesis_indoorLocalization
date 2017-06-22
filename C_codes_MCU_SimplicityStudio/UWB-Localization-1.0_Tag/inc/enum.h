/*
 * enum.h
 *
 *  Created on: 16.08.2016
 *      Author: Guillermo
 */

#ifndef ENUM_H_
#define ENUM_H_

enum states_tag {
			STATE_SEND_BLINK = 0,
			STATE_RECEIVER_ON,
			STATE_RECEIVE_MODE,
			STATE_POLL_RECEIVE,
			STATE_RESPONSE_TRANSMIT,
			STATE_WAIT_FINAL_RECEIVE,
			STATE_FINAL_RECEIVE
		};

enum states_node {
			STATE_POLL_TRANSMIT = 0,
			STATE_WAIT_RESPONSE_RECEIVE,
			STATE_RESPONSE_RECEIVE,
			STATE_FINAL_TRANSMIT
};


enum states_DWM1000 {
			STATE_IDLE = 0,
			STATE_TRANSMIT,
			STATE_RECEIVE,
			STATE_SLEEP
};

enum states_localization {
			STATE_WAKE_UP = 0,
			STATE_UWB
};

#endif /* ENUM_H_ */
