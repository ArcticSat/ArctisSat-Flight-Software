/*
 * eps.c
 *
 *  Created on: Dec. 6, 2022
 *      Author: jpmckoy
 */

#include "application/eps.h"
#include "drivers/protocol/can.h"

void HandlePowerCommand(telemetryPacket_t * cmd_pkt)
{
	CANMessage_t cmd = {0};
	cmd.id = POW_TXID;
	switch(cmd_pkt->telem_id){

		case POWER_READ_TEMP_CMD:{ // Read temperature value command
			cmd.dlc = 2;
			cmd.data[0] = POWER_READ_TEMP_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_READ_SOLAR_CURRENT_CMD:{ // Read solar current command
			cmd.dlc = 2;
			cmd.data[0] = POWER_READ_SOLAR_CURRENT_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_READ_LOAD_CURRENT_CMD:{ // Read solar current command
			cmd.dlc = 2;
			cmd.data[0] = POWER_READ_LOAD_CURRENT_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_READ_MSB_VOLTAGE_CMD:{ // Read solar current command
			cmd.dlc = 1;
			cmd.data[0] = POWER_READ_MSB_VOLTAGE_CMD;
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_GET_BATTERY_SOC_CMD:{ // Read solar current command
			cmd.dlc = 1;
			cmd.data[0] = POWER_GET_BATTERY_SOC_CMD;
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_GET_SA_CHARGE_STATE_CMD:{ // Read solar current command
			cmd.dlc = 1;
			cmd.data[0] = POWER_GET_SA_CHARGE_STATE_CMD;
			CAN_transmit_message(&cmd);
			break;
		}
	//		case POWER_GET_BOOT_COUNT:{ // Read solar current command
	//			CANMessage_t cmd = {0};
	//			cmd.id = POW_TXID;
	//			cmd.dlc = 1;
	//			cmd.data[0] = POWER_GET_BOOT_COUNT_CMD;
	//			CAN_transmit_message(&cmd);
	//			break;
	//		}
		case POWER_SET_LOAD_OFF_CMD:{
			cmd.dlc = 2;
			cmd.data[0] = POWER_SET_LOAD_OFF_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_SET_LOAD_ON_CMD:{
			cmd.dlc = 2;
			cmd.data[0] = POWER_SET_LOAD_ON_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_SET_SOLAR_OFF_CMD:{
			cmd.dlc = 2;
			cmd.data[0] = POWER_SET_SOLAR_OFF_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_SET_SOLAR_ON_CMD:{
			cmd.dlc = 2;
			cmd.data[0] = POWER_SET_SOLAR_ON_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_SET_POW_MODE_CMD:{
			cmd.dlc = 2;
			cmd.data[0] = POWER_SET_POW_MODE_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_AIT_SET_BATTERY_SOC_CMD:{
			float soc;
			memcpy(&soc,cmd_pkt->data,sizeof(float));
			cmd.dlc = 5;
			cmd.data[0] = POWER_AIT_SET_BATTERY_SOC_CMD;
			memcpy(&cmd.data[1],&cmd_pkt->data[0],sizeof(float));
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_FRAM_GET_OPMODE_CMD: {
			cmd.dlc = 1;
			cmd.data[0] = POWER_FRAM_GET_OPMODE_CMD;
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_FRAM_GET_SOC_CMD: {
			cmd.dlc = 1;
			cmd.data[0] = POWER_FRAM_GET_SOC_CMD;
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_FRAM_LOG_OPMODE_CMD:{
			cmd.dlc = 2;
			cmd.data[0] = POWER_FRAM_LOG_OPMODE_CMD;
			cmd.data[1] = cmd_pkt->data[0];
			CAN_transmit_message(&cmd);
			break;
		}
		case POWER_FRAM_LOG_SOC_CMD:{
			float soc;
			memcpy(&soc,cmd_pkt->data,sizeof(float));
			cmd.dlc = 5;
			cmd.data[0] = POWER_FRAM_LOG_SOC_CMD;
			memcpy(&cmd.data[1],cmd_pkt->data,sizeof(float));
	//						memcpy(&cmd.data[1],&cmd_pkt->data[0],sizeof(float));
			CAN_transmit_message(&cmd);
			break;
		}
	} // switch(cmd_pkt->telem_id)
}
