/*
 * cdh.h
 *
 *  Created on: Dec. 6, 2022
 *      Author: jpmck
 */

#ifndef INCLUDE_APPLICATION_CDH_H_
#define INCLUDE_APPLICATION_CDH_H_


//#include "main.h"
//#include <stdint.h>
#include "tasks/telemetry.h"


void HandleCdhCommand(telemetryPacket_t * cmd_pkt);
void vCanServer(void * pvParameters);


#endif /* INCLUDE_APPLICATION_CDH_H_ */
