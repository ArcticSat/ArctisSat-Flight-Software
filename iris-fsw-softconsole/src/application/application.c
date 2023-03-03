/*
 * application.c
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */


#include "application/application.h"

//bool ls_rst_status[NUM_LOAD_SWITCHES];
//
void HandleTm(csp_conn_t * conn, csp_packet_t * packet)
{
	int src = csp_conn_src(conn);
	switch(src){
		case PAYLOAD_CSP_ADDRESS:{
			HandlePayloadTlm(conn,packet);
			break;
		}
		default:{
			break;
		}
	}
}
//
//bool GetLsRstStatus(uint8_t switchNumber)
//{
//	return ls_rst_status[switchNumber];
//}
//
//bool ResetLoadSwitch(uint8_t switchNumber)
//{
//	ls_rst_status[switchNumber] = false;
//}
//
//void InitApplication(void)
//{
//	int i;
//	for(i=0; i < NUM_LOAD_SWITCHES; i++)
//		ls_rst_status[i] = true;
//}
