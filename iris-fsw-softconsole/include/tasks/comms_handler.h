/*
 * comms_handler.h
 *
 *  Created on: Jan 23, 2025
 *      Author: brend
 */

#ifndef INCLUDE_TASKS_COMMS_HANDLER_H_
#define INCLUDE_TASKS_COMMS_HANDLER_H_

#include <stdint.h>

void commsHandlerTask();
void commsReceiverTask();
void commsTransmitterTask();
void sendRawData(uint8_t* data, uint16_t type, int len);

#endif /* INCLUDE_TASKS_COMMS_HANDLER_H_ */
