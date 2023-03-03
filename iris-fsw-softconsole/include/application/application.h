/*
 * application.h
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */

#include <stdbool.h>
#include <stdint.h>
#include "tasks/telemetry.h"
#include "csp/csp.h"


#include "application/cdh.h"
#include "application/eps.h"
#include "application/payload.h"
#include "application/adcs.h"

// Enums
//typedef enum
//{
//    LS_HTR,
//    LS_ADCS,
//    LS_COMS,
//    LS_CDH,
//    LS_PLD,
//    LS_DPL_A,
//    LS_DPL_S,
//    NUM_LOAD_SWITCHES
//} LoadSwitchNumbers_t;

// Functions
void HandleTm(csp_conn_t * conn, csp_packet_t * packet);
bool GetLsRstStatus(uint8_t switchNumber);
bool ResetLoadSwitch(uint8_t switchNumber);
void InitApplication(void);
