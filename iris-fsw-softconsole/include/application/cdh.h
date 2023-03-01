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

//This will try and run an immediate command.
//return:   0  > command executed.
//         -1 -> Not an immediate command. So the sche
int handleCdhImmediateCommand(telemetryPacket_t * cmd_pkt, csp_conn_t * conn);

void vCanServerBasic(void * pvParameters);
void vCanServer(void * pvParameters);


#endif /* INCLUDE_APPLICATION_CDH_H_ */
