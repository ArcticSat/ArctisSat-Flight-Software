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
#include "tasks/telemetry.h"

//Define the csp address of all devices in the network.
#define POWER_CSP_ADDRESS	2
#define COMMS_CSP_ADDRESS	3
#define CDH_CSP_ADDRESS		4
#define PAYLOAD_CSP_ADDRESS	5
#define GROUND_CSP_ADDRESS	9


//Define the ports we're using
#define CSP_CMD_PORT	7
#define CSP_COMMS_PASSTHROUGH 11
#define CSP_TELEM_PORT	8
#define CSP_UPDATE_PORT	9

// Maximum CSP packet size
#define DLC_MAX 256 // TBC

/***********************************************************/
//Telemetry IDs
typedef enum
{
	/*** POWER TELEMETRY ***/
    POWER_READ_TEMP_ID = 0,
    POWER_READ_SOLAR_CURRENT_ID,
    POWER_READ_LOAD_CURRENT_ID,
    POWER_READ_MSB_VOLTAGE_ID,
    POWER_GET_BATTERY_SOC_ID,
    POWER_GET_SA_CHARGE_STATE_ID,
    POWER_GET_BOOT_COUNT_ID,
    POWER_RESET_LOAD_SWITCH_ID,
    POWER_FRAM_GET_OPMODE_ID,
    POWER_FRAM_GET_SOC_ID,
    POWER_FRAM_GET_BOOT_COUNT_ID,
    POWER_READ_ADC_A_ID,
    POWER_READ_ADC_B_ID,
    // Additional telemetry
    POWER_FRAM_RESULT_ID,
    POWER_CDH_SHUTDOWN_ID,
    POWER_GET_SA_CURRENT_THRESHOLD_ID,
    POWER_GET_ALL_LOAD_CURRENTS_ID,
    POWER_GET_ALL_SOLAR_ARRAY_CURRENTS_ID,
    POWER_TELEMETRY_END,
	/*** PAYLOAD TELEMETRY ***/
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
	PAYLOAD_CMD_CONFIRM_ID,
	PAYLOAD_CMD_ERROR_ID,
	PAYLOAD_I2C_REG_READ16_CMD,
	PAYLIAD_I2C_RECEIVE_CMD,
	PAYLOAD_CAMERA_SELECT_CMD,
	PAYLOAD_WRITE_CUSTOM_REGLIST_CMD,
	PAYLOAD_REGLIST_CONFIG_CMD,
	PAYLOAD_GET_IMAGE_TX_DEST_ADDR_ID,
	PAYLOAD_TELEMETRY_END,
	/*** ADCS TELEMETRY ***/
	ADCS_MESAUREMENT_GYRO_ID,
	ADCS_MESAUREMENT_MAGNETOMETER_ID,
	ADCS_GYROSCOPE_DATA_ID,
	ADCS_MAGNETOMETER_DATA_ID,
	ADCS_SS_META_ID,
	ADCS_SS_RAW_ID,
	ADCS_SS_ANGLE_ID,
	ADCS_SS_UNIT_VECTOR_ID,
	ADCS_GET_ECLIPSE_TIME_ID,
	ADCS_TELEMETRY_END,
	/*** CDH TELEMETRY ***/
	CDH_TIME_ID,
	CDH_BOARD_TEMP_ID,
	CDH_TTT_LIST_ID,
	CDH_MSG_ID,
	CDH_FW_STATE_ID,
	CDH_FW_ACK_ID,
	CDH_FW_RQST_ID,
	// Memory manager telemetry
	CDH_SPACECRAFT_STATUS_ID,
	// Control telemetry
	CDH_DETUMBLING_TM_ID,
	CDH_SUN_POINTING_ID,
	CDH_HW_STATUS_ID,
	// CDH telemetry end
	CDH_TELEMETRY_END,
	/*** EVENT TELEMETRY ***/
	EVENT_ID,
	/*** GROUND TELEMETRY ***/
	GND_FRAME_ID,
	GND_TELEMETRY_END,
} TelemetryId_t;
// Command IDs
typedef enum
{
	/*** POWER COMMANDS ***/
	// POWER READ COMMANDS
	POWER_READ_TEMP_CMD = 0,
	POWER_READ_SOLAR_CURRENT_CMD,
	POWER_READ_LOAD_CURRENT_CMD,
	POWER_READ_MSB_VOLTAGE_CMD,
	// POWER GET COMMANDS
	POWER_GET_BATTERY_SOC_CMD,
	POWER_GET_SA_CHARGE_STATE_CMD,
	POWER_GET_BOOT_COUNT_CMD,
	// POWER SET COMMANDS
	POWER_SET_LOAD_OFF_CMD,
	POWER_SET_LOAD_ON_CMD,
	POWER_SET_SOLAR_OFF_CMD,
	POWER_SET_SOLAR_ON_CMD,
	POWER_SET_POW_MODE_CMD,
	// POWER RESET COMMAND
	POWER_RESET_LOAD_SWITCH_CMD,
	// POWER FRAM commands
	POWER_FRAM_GET_OPMODE_CMD,
	POWER_FRAM_GET_SOC_CMD,
	POWER_FRAM_GET_BOOT_COUNT_CMD,
	POWER_FRAM_LOG_OPMODE_CMD,
	POWER_FRAM_LOG_SOC_CMD,
	POWER_FRAM_LOG_BOOT_COUNT_CMD,
    POWER_GET_ALL_LOAD_CURRENTS_CMD,
    POWER_GET_ALL_SOLAR_ARRAY_CURRENTS_CMD,
	// POWER ADC commands
	POWER_READ_ADC_A_CMD,
	POWER_READ_ADC_B_CMD,
	// POWER AIT COMMANDS
	POWER_AIT_SET_BATTERY_SOC_CMD,
    // Additional commands
    POWER_SET_CAN_TX_HEADER_CMD,
    POWER_SET_SECONDARY_TXID_CMD,
    POWER_SET_CAN_SECONDARY_HEADER_PARAM_CMD,
    POWER_SET_SECONDARY_RXID_CMD,
    POWER_GET_SA_CURRENT_THRESHOLD_CMD,
    POWER_SET_SA_CURRENT_THRESHOLD_CMD,
	// POWER COMMANDS END
	POWER_COMMANDS_END,
	/*** PAYLOAD COMMANDS ***/
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
	PAYLOAD_DELETE_IMG,
	PAYLOAD_TRANSFER_META,
	PAYLOAD_SWAP_MODE,
	PAYLOAD_TRANSFER_IMAGE_CMD,
	PAYLOAD_FORMAT_FS_CMD,
	PAYLOAD_SET_AUTO_EXPOSURE_CMD,
	PAYLOAD_GET_IMAGE_TX_DEST_ADDR_CMD,
	PAYLOAD_SET_IMAGE_TX_DEST_ADDR_CMD,
	PAYLOAD_COMMANDS_END,
	/*** ADCS COMMANDS ***/
	ADCS_INIT_CMD,
	ADCS_TX_RX_CMD,
	ADCS_PING_CMD,
	ADCS_SPI_SYNC_CMD,
	ADCS_SET_TR_STATE_CMD,
	ADCS_SET_TR_POLARITY_CMD,
	ADCS_SET_TR_PWM_CMD,
	ADCS_SET_GYRO_I2C_ADDRESS_CMD,
	ADCS_GET_MEASUREMENT_GYRO_GENERIC_CMD,
	ADCS_GET_MEASUREMENT_GYRO_CMD,
	ADCS_GET_MEASUREMENT_MAGNETOMETER_CMD,
	ADCS_SUN_SENSOR_SELECT_CMD,
	ADCS_GET_MEASUREMENT_SUN_CMD,
	ADCS_GET_SUN_POINTING_TM_CMD,
	ADCS_GET_ECLIPSE_TIME_CMD,
	ADCS_SET_ECLIPSE_TIME_CMD,
	ADCS_COMMANDS_END,
	/*** CDH COMMANDS ***/
	CDH_SCHEDULE_TTT_CMD,
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
	CDH_FW_RX_PROGRESS_CMD,
	CDH_FW_PUT_DATA_2_CMD,
	CDH_CHECKSUM_FILE_PART_CMD,
	CDH_FILE_WRITE_CMD,
	CDH_FILE_INSERT_CMD,
	CDH_FILE_DELETE_CMD,
	// Memory manager commands
	CDH_GET_SPACECRAFT_STATUS_CMD,
	CDH_SET_DEPLOYMENT_STARTUP_STATE_CMD,
	CDH_SET_DETUMBLING_STARTUP_STATE_CMD,
	CDH_CLEAR_REBOOT_REASON_CMD,
	CDH_GET_HW_STATUS_CMD,
	CDH_FORCE_FW_STATE_CMD,
	CDH_SET_FW_ARM_TIMEOUT_CMD,
	CDH_WATCHDOG_DISABLE_CMD,
	CDH_RQST_PWR_RESET_LOAD_CMD,

	CDH_COMMANDS_END,
	/*** GROUND COMMANDS ***/
	GND_TM_VERBOSITY_CMD,
	GND_TELEMETRY_REQUEST_CMD,
	/*** ALL COMMANDS END ***/
	IMMED_COMMANDS_END,
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
void set_csp_init(int state);
int is_csp_up();

/**********************************************************/

#endif /* INC_TELEMETRY_H_ */
