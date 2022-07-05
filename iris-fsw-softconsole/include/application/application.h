/*
 * application.h
 *
 *  Created on: Jul. 4, 2022
 *      Author: jpmck
 */

#include <stdbool.h>
#include <stdint.h>
#include "tasks/telemetry.h"

// Enums
typedef enum
{
    LS_HTR,
    LS_ADCS,
    LS_COMS,
    LS_CDH,
    LS_PLD,
    LS_DPL_A,
    LS_DPL_S,
    NUM_LOAD_SWITCHES
} LoadSwitchNumbers_t;

// Functions
void HandleTm(telemetryPacket_t * tm_pkt);
bool GetLsRstStatus(uint8_t switchNumber);
bool ResetLoadSwitch(uint8_t switchNumber);
void InitApplication(void);
