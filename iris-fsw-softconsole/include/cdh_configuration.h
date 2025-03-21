/*
 * cdh_configuration.h
 *
 *  Created on: Mar. 30, 2023
 *      Author: Jayden McKoy
 */

#ifndef INCLUDE_CDH_CONFIGURATION_H_
#define INCLUDE_CDH_CONFIGURATION_H_

/*** Flight parameters ***/
// Set these
#define FLIGHT_MODEL_CONFIGURATION
//#define INIT_COMMS
//#define DEPLOYMENT_CONFIG

//#define INCLUDE_TASK_SUN_POINTING
//#define INCLUDE_TASK_TTT
#define INCLUDE_TASK_CAN_SERVER
//#define INCLUDE_TASK_FW_MANAGER

// Un-set these
//#define ENGINEERING_MODEL_CONFIGURATION
//#define MAKER2_DEVKIT_CONFIGURATION
//#define DEBUG_TELEMETRY
//#define SUN_POINTING_DEBUG_TELEMETRY
//#define BACKPANEL_DEBUG_TELEMETRY


/*** Optional parameters ***/
//#define BACKWARD_POLL_BACK_PANELS

#endif /* INCLUDE_CDH_CONFIGURATION_H_ */
