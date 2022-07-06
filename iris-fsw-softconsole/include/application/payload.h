/*
 * payload.h
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */

#ifndef INCLUDE_APPLICATION_PAYLOAD_H_
#define INCLUDE_APPLICATION_PAYLOAD_H_

#include "tasks/telemetry.h"
#include "csp/csp.h"

void HandlePayloadTlm(csp_conn_t * conn, csp_packet_t * packet);


#endif /* INCLUDE_APPLICATION_PAYLOAD_H_ */
