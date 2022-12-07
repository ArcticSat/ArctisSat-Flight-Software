/*
 * telemetry.h
 *	Note telemetry and command is used interchangably here.
 *  Created on: Mar. 17, 2021
 *      Author: Joseph Howarth
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

//#include <stdint.h>
#include "drivers/device/rtc/rtc_common.h"
#include <csp/csp.h>

//Define the csp address of all devices in the network.
#define POWER_CSP_ADDRESS	2
#define COMMS_CSP_ADDRESS	3
#define CDH_CSP_ADDRESS		4
#define PAYLOAD_CSP_ADDRESS	5
#define GROUND_CSP_ADDRESS	9


//Define the ports we're using
#define CSP_CMD_PORT	7
#define CSP_TELEM_PORT	8
#define CSP_UPDATE_PORT	9

// Maximum CSP packet size
#define DLC_MAX 256

/***********************************************************/
//Telemetry IDs
typedef enum
{
	// CDH TELEMETRY
	CDH_TIME_ID =0,
	CDH_BOARD_TEMP_ID,
	CDH_TTT_LIST_ID,
	CDH_MSG_ID,
	CDH_FW_STATE_ID,
	CDH_TELEMETRY_END,
	// POWER TELEMETRY
	POWER_READ_TEMP_ID,
	POWER_READ_SOLAR_CURRENT_ID,
	POWER_READ_LOAD_CURRENT_ID,
	POWER_READ_MSB_VOLTAGE_ID,
	POWER_GET_BATTERY_SOC_ID,
	POWER_GET_ECLIPSE_ID,
	POWER_GET_BOOT_COUNT_ID,
//	POWER_SET_LOAD_OFF_ID,
//	POWER_SET_LOAD_ON_ID,
//	POWER_RESET_LOAD_SWITCH_ID,
//	POWER_SET_SOLAR_OFF_ID,
//	POWER_SET_SOLAR_ON_ID,
//	POWER_SET_MODE_ID,
//	POWER_AIT_SET_BATTERY_SOC_ID,
//	POWER_AIT_SET_ECLIPSE_ID,
	POWER_TELEMETRY_END,
	// PAYLOAD TELEMETRY
	PAYLOAD_POWER_GOOD_ID,
	PAYLOAD_BOARD_TEMP_ID,
	PAYLOAD_SAMPLE_TEMP_ID,
	PAYLOAD_FULL_IMAGE_ID,
	PAYLOAD_SAMPLE_LOC_ID,
	PAYLOAD_CAMERA_TIME_ID,
	PAYLOAD_ERROR_ID,
	PAYLOAD_FULL_IMAGE_RX, //Only for debugging remove later
	PAYLOAD_FILE_LIST_ID,
	PAYLOAD_META_ID,
	PAYLOAD_IMAGE_INFO,
	PAYLOAD_IMAGE_CAPTURE_ERROR_ID,
	PAYLOAD_ACK,
	PAYLOAD_MSG_ID,
	PAYLOAD_TELEMETRY_END,
	// ADCS TELEMETRY
	ADCS_MESAUREMENT_GYRO_ID,
	ADCS_MESAUREMENT_MAGNETOMETER_ID,
	ADCS_MESAUREMENT_SUN_ID,
	ADCS_TELEMETRY_END,
	// Event ID
	EVENT_ID,
	// GROUND TELEMETRY
	GND_FRAME_ID,
	GND_TELEMETRY_END,
} TelemetryId_t;
// Command IDs
typedef enum
{
	// CDH COMMANDS
	CDH_SCHEDULE_TTT_CMD =0,
	CDH_SET_TIME_CMD,
	CDH_GET_TIME_CMD,
	CDH_DOWNLOAD_IMAGE_CMD,
	CDH_DOWNLOAD_LOG_FILE_CMD,
	CDH_LIST_FILES_CMD,
	// FW Manager Commands:
	CDH_LIST_FW_CMD,
	CDH_FW_IDLE_CMD,
	CDH_FW_RX_FW_CMD,
	CDH_FW_PRE_VER_CMD,
	CDH_FW_ARM_CMD,
	CDH_FW_EXECUTE_CMD,
	CDH_FW_EXECUTE_CONFIRM_CMD,
	CDH_FW_POST_VER_CMD,
	CDH_FW_PUT_DATA_CMD,
	CDH_FW_GET_STATE_CMD,
	CDH_CHECKSUM_FILE_CMD,
	CDH_MV_FILE_CMD,
	CDH_RM_FILE_CMD,
	CDH_CP_FILE_CMD,
	CDH_CHECKSUM_PGRM_FLASH_CMD,
	CDH_CP_TO_PGRM_FLASH_CMD,
	CDH_GET_SW_VER_CMD,
	CDH_GET_DES_VER_CMD,
	CDH_GET_SPI_DIR_CMD,
	CDH_GET_FS_FREE_SPACE_CMD,
	CDH_FW_UPDATE_SPI_DIR_CMD,
	CDH_FW_CREATE_SPI_DIR_CMD,
	CDH_WRITE_PROG_FLASH_CMD,
	CDH_ERASE_PROG_FLASH_CMD,
	CDH_RESET_FW_MNGR_CMD,
	CDH_FW_SET_CHECKSUM_CMD,
	CDH_FW_SET_DESVER_CMD,
	CDH_FORMAT_FS_CMD,
	CDH_RESET_SYSTEM_CMD,
	CDH_COMMANDS_END,
	// POWER COMMANDS
	POWER_READ_TEMP_CMD,
	POWER_READ_SOLAR_CURRENT_CMD,
	POWER_READ_LOAD_CURRENT_CMD,
	POWER_READ_MSB_VOLTAGE_CMD,
	POWER_GET_BATTERY_SOC_CMD,
	POWER_GET_ECLIPSE_CMD,
	POWER_GET_BOOT_COUNT_CMD,
	POWER_SET_LOAD_OFF_CMD,
	POWER_SET_LOAD_ON_CMD,
	POWER_SET_SOLAR_OFF_CMD,
	POWER_SET_SOLAR_ON_CMD,
	POWER_SET_POW_MODE_CMD,
	POWER_RESET_LOAD_SWITCH_CMD,
	POWER_AIT_SET_BATTERY_SOC_CMD,
	POWER_AIT_SET_ECLIPSE,
	POWER_COMMANDS_END,
	// PAYLOAD COMMANDS
	PAYLOAD_POWER_GOOD_CMD,
	PAYLOAD_BOARD_TEMP_CMD,
	PAYLOAD_SAMPLE_TEMP_CMD,
	PAYLOAD_FULL_IMAGE_CMD,
	PAYLOAD_TAKE_IMAGE_CAM1_CMD,
	PAYLOAD_TAKE_IMAGE_CAM2_CMD,
	PAYLOAD_CAM1_ON_CMD,
	PAYLOAD_CAM2_ON_CMD,
	PAYLOAD_CAM1_OFF_CMD,
	PAYLOAD_CAM2_OFF_CMD,
	PAYLOAD_CAM1_RESET_CMD,
	PAYLOAD_CAM2_RESET_CMD,
	PAYLOAD_ENTER_LOW_POWER_CMD,
	PAYLOAD_EXIT_LOW_POWER_CMD,
	PAYLOAD_FILE_LIST_CMD,
	PAYLOAD_SHUTDOWN_CMD,
	PAYLOAD_TURNONCAM1_CMD,
	PAYLOAD_CAMERA_SENSOR_INIT,
	PAYLOAD_CAMERA_HANDSHAKE,
	PAYLOAD_CAMERA_SET_I2C_WRITE_ADDRESS,
	PAYLOAD_CAMERA_SET_I2C_READ_ADDRESS,
	PAYLOAD_CAMERA_SET_I2C_TIMEOUT,
	PAYLOAD_CAMERA_I2C_TRANSMIT,
	PAYLOAD_CAMERA_I2C_RECEIVE,
	PAYLOAD_CAMERA_WRITE_REG_LIST,
	PAYLOAD_CAMERA_TEST_INIT,
	PAYLOAD_MOUNT_FS,
	PAYLOAD_UNMOUNT_FS,
	PAYLOAD_RESTART_FS,
	PAYLOAD_COMMANDS_END,
	// ADCS COMMANDS
	ADCS_INIT_CMD,
	ADCS_ADCS_TX_RX_CMD,
	ADCS_PING_CMD,
	ADCS_SPI_SYNC_CMD,
	ADCS_SET_TR_STATE_CMD,
	ADCS_SET_TR_POLARITY_CMD,
	ADCS_SET_TR_PWM_CMD,
	ADCS_GET_MEASUREMENT_GYRO_CMD,
	ADCS_GET_MEASUREMENT_MAGNETOMETER_CMD,
	ADCS_GET_MEASUREMENT_SUN_CMD,
	ADCS_COMMANDS_END,
} CommandId_t;
// Task IDs
typedef enum {
	// CDH tasks,
	TASK_CDH_END,
	// Power tasks,
	TASK_POWER_READ_TEMP,
	TASK_POWER_READ_SOLAR_CURRENT,
	TASK_POWER_READ_LOAD_CURRENT,
	TASK_POWER_READ_MSB_VOLTAGE,
	TASK_POWER_GET_BATTERY_SOC,
	TASK_POWER_GET_ECLIPSE,
	TASK_POWER_GET_BOOT_COUNT,
	TASK_POWER_SET_LOAD_OFF,
	TASK_POWER_SET_LOAD_ON,
	TASK_POWER_SET_SOLAR_OFF,
	TASK_POWER_SET_SOLAR_ON,
	TASK_POWER_SET_MODE,
	TASK_AIT_POWER_SET_BATTERY_SOC,
	TASK_AIT_POWER_SET_ECLIPSE,
	TASK_POWER_END,
	// Payload tasks
	TASK_TAKE_IMAGE,
	// ADCS tasks,
	TASK_ADCS_INIT,
	TASK_ADCS_ADCS_TX_RX,
	TASK_ADCS_PING,
	TASK_ADCS_SPI_SYNC,
	TASK_ADCS_SET_TR_STATE,
	TASK_ADCS_SET_TR_POLARITY,
	TASK_ADCS_SET_TR_PWM,
	TASK_ADCS_GET_MEASUREMENT_GYRO,
	TASK_ADCS_GET_MEASUREMENT_MAGNETOMETER,
	TASK_ADCS_GET_MEASUREMENT_SUN,
	TASK_ADCS_END,
} TaskId_t;


/**********************************************************/

//typedef struct{
//	uint8_t second;
//	uint8_t minute;
//	uint8_t hour;
//	uint8_t day;
//	uint8_t month;
//	uint8_t year;
//	uint8_t weekday;
//	uint8_t week;
//
//}Calendar_t;


typedef struct{

	Calendar_t timestamp;
	uint8_t telem_id;		//Make sure there is less than 255 commands/telemetry ids for any subsystem. Or change to uint16_t.
	uint8_t length;
	uint8_t* data;

} telemetryPacket_t;


/**********************************************************/
void unpackTelemetry(uint8_t * data, telemetryPacket_t* output);//Unpacks the telemetry into the telemetry packet struct.
void sendTelemetry(telemetryPacket_t * packet);//Sends telemetry to CDH.
void sendTelemetry_direct(telemetryPacket_t * packet,csp_conn_t * conn); //For directly responding to a message.
void sendCommand(telemetryPacket_t * packet,uint8_t addr);//Sends a cmd packet to the cmd port of the subsytem at address addr.
void sendTelemetryAddr(telemetryPacket_t * packet,uint8_t addr); //Sends telemetry directly to a subsystem.
void printMsg(char * msg);
int printf(const char *fmt, ...);

/**********************************************************/

#endif /* INC_TELEMETRY_H_ */
