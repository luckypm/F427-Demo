/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include "aq.h"
#ifdef USE_MAVLINK
#include "aq_mavlink.h"
// lots of warnings coming from mavlink
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "mavlink.h"
#pragma GCC diagnostic pop
#include "config.h"
#include "imu.h"
//#include "radio.h"
#include "gps.h"
#include "flash.h"
//#include "motors.h"
//#include "control.h"
//#include "nav.h"
#include "math.h"
#include "util.h"
#include "rcc.h"
#include "supervisor.h"
#include "nav_ukf.h"
//#include "analog.h"
#include "comm.h"
//#include "gimbal.h"
#include "d_imu.h"
#include "run.h"
#include <string.h>
#include <stdio.h>

mavlinkStruct_t mavlinkData;
mavlink_system_t mavlink_system;

void mavlinkSendPacket(mavlink_channel_t chan, const uint8_t *buf, uint16_t len) {
    commTxBuf_t *txBuf;
    uint8_t *ptr;
    int i;

    txBuf = commGetTxBuf(COMM_STREAM_TYPE_MAVLINK, len);
    // cannot block, must fail
    if (txBuf != 0) {
	ptr = &txBuf->buf;

	for (i = 0; i < len; i++)
	    *ptr++ = *buf++;

	commSendTxBuf(txBuf, len);
    }
}

void mavlinkWpReached(uint16_t seqId) {
    mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0, seqId);
}

void mavlinkWpAnnounceCurrent(uint16_t seqId) {
    mavlink_msg_mission_current_send(MAVLINK_COMM_0, seqId);
}

void mavlinkSendNotice(const char *s) {
    mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, (const char *)s);
}

void mavlinkToggleStreams(uint8_t enable) {
    for (uint8_t i = 0; i < AQMAVLINK_TOTAL_STREAMS; i++)
	mavlinkData.streams[i].enable = enable && mavlinkData.streams[i].interval;
}


void mavlinkDo(void) {
    static unsigned long lastMicros = 0;
    unsigned long micros,statusInterval;
    int8_t streamAll, i;

    micros = timerMicros();

    // handle rollover
    if (micros < lastMicros) {
	mavlinkData.nextHeartbeat = 0;
	mavlinkData.nextParam = 0;
	for (uint8_t i=0; i < AQMAVLINK_TOTAL_STREAMS; ++i)
	    mavlinkData.streams[i].next = 0;
    }
    // heartbeat
    if (mavlinkData.nextHeartbeat < micros) {
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0,0, MAV_AUTOPILOT_AUTOQUAD, 0, 0, 0);
	mavlinkData.nextHeartbeat = micros + AQMAVLINK_HEARTBEAT_INTERVAL;
    }

    // send streams

    // first check if "ALL" stream is requested
    if ((streamAll = mavlinkData.streams[MAV_DATA_STREAM_ALL].enable && mavlinkData.streams[MAV_DATA_STREAM_ALL].next < micros))
	mavlinkData.streams[MAV_DATA_STREAM_ALL].next = micros + mavlinkData.streams[MAV_DATA_STREAM_ALL].interval;

    // status
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].enable && mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].next < micros)) {
	// calculate idle time
	statusInterval = streamAll ? mavlinkData.streams[MAV_DATA_STREAM_ALL].interval : mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].interval;

	mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].next = micros + mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].interval;
    }
    // raw sensors
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].enable && mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].next < micros)) {
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, micros, IMU_ACCX*1000.0f, IMU_ACCY*1000.0f, IMU_ACCZ*1000.0f, IMU_RATEX*1000.0f, IMU_RATEY*1000.0f, IMU_RATEZ*1000.0f,
		IMU_MAGX*1000.0f, IMU_MAGY*1000.0f, IMU_MAGZ*1000.0f);
	mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0, micros, AQ_PRESSURE*0.01f, 0.0f, IMU_TEMP*100);
	mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].next = micros + mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].interval;
    }
    // position -- gps and ukf
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_POSITION].enable && mavlinkData.streams[MAV_DATA_STREAM_POSITION].next < micros)) {
	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, micros, 0, gpsData.lat*(double)1e7, gpsData.lon*(double)1e7, gpsData.height*1e3,
		gpsData.hAcc*100, gpsData.vAcc*100, gpsData.speed*100, gpsData.heading, 255);
	mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, micros, UKF_POSN, UKF_POSE, -UKF_POSD, UKF_VELN, UKF_VELE, -VELOCITYD);
	mavlinkData.streams[MAV_DATA_STREAM_POSITION].next = micros + mavlinkData.streams[MAV_DATA_STREAM_POSITION].interval;
    }
    // rc channels and pwm outputs (would be nice to separate these)
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].enable && mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].next < micros)) {
	mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].next = micros + mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].interval / 2;
    }
    // raw controller -- attitude and nav data
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].enable && mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].next < micros)) {
	mavlink_msg_attitude_send(MAVLINK_COMM_0, micros, AQ_ROLL*DEG_TO_RAD, AQ_PITCH*DEG_TO_RAD, AQ_YAW*DEG_TO_RAD, -(IMU_RATEX - UKF_GYO_BIAS_X)*DEG_TO_RAD,
		(IMU_RATEY - UKF_GYO_BIAS_Y)*DEG_TO_RAD, (IMU_RATEZ - UKF_GYO_BIAS_Z)*DEG_TO_RAD);
	mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].next = micros + mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].interval;
    }
    // EXTRA3 stream -- AQ custom telemetry
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].enable && mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].next < micros)) {
	for (i=0; i < AQMAV_DATASET_ENUM_END; ++i) {
	    if (!mavlinkData.customDatasets[i])
		continue;
	    switch(i) {
	    case AQMAV_DATASET_LEGACY1 :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, AQ_ROLL, AQ_PITCH, AQ_YAW, IMU_RATEX, IMU_RATEY, IMU_RATEZ, IMU_ACCX, IMU_ACCY, IMU_ACCZ, IMU_MAGX, IMU_MAGY, IMU_MAGZ,
			0, AQ_PRESSURE, IMU_TEMP, ALTITUDE, 0, UKF_POSN, UKF_POSE, UKF_POSD);
		break;
	    case AQMAV_DATASET_LEGACY2 :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, gpsData.lat, gpsData.lon, gpsData.hAcc, gpsData.heading, gpsData.height, gpsData.pDOP,
			0, 0, 0,0,0, UKF_VELN, UKF_VELE, VELOCITYD,
			0, UKF_ACC_BIAS_X, UKF_ACC_BIAS_Y, UKF_ACC_BIAS_Z, 0, 0);
		break;
	    case AQMAV_DATASET_LEGACY3 :
		
		break;
	    case AQMAV_DATASET_GPS_XTRA :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, gpsData.pDOP, gpsData.tDOP, gpsData.sAcc, gpsData.cAcc, gpsData.velN, gpsData.velE, gpsData.velD,
			gpsData.lastPosUpdate, gpsData.lastMessage, 0,0,0,0,0,0,0,0,0,0,0);
		break;
	    case AQMAV_DATASET_UKF_XTRA :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, UKF_GYO_BIAS_X, UKF_GYO_BIAS_Y, UKF_GYO_BIAS_Z, UKF_ACC_BIAS_X, UKF_ACC_BIAS_Y, UKF_ACC_BIAS_Z, UKF_Q1, UKF_Q2, UKF_Q3, UKF_Q4,
			0,0,0,0,0,0,0,0,0,0);
		break;
	    case AQMAV_DATASET_SUPERVISOR :
		break;
	    case AQMAV_DATASET_STACKSFREE :

		break;
	    case AQMAV_DATASET_GIMBAL :

		break;
	    }
	}

	mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].next = micros + mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].interval;
    }
    // end streams

    // list all requested/remaining parameters
    if (mavlinkData.currentParam < CONFIG_NUM_PARAMS && mavlinkData.nextParam < micros) {
	mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameterStrings[mavlinkData.currentParam], p[mavlinkData.currentParam], MAVLINK_TYPE_FLOAT, CONFIG_NUM_PARAMS, mavlinkData.currentParam);
	mavlinkData.currentParam++;
	mavlinkData.nextParam = micros + AQMAVLINK_PARAM_INTERVAL;
    }

}


void mavlinkDoCommand(mavlink_message_t *msg) {
    uint16_t command;
    float param, param2, param3;
    uint8_t enable, i, compId,
	ack = MAV_CMD_ACK_ERR_NOT_SUPPORTED;

    command = mavlink_msg_command_long_get_command(msg);
    compId = mavlink_msg_command_long_get_target_component(msg);

    switch (command) {

	case MAV_CMD_PREFLIGHT_CALIBRATION:
	    if (!(supervisorData.state & STATE_ARMED)) {
		param = mavlink_msg_command_long_get_param2(msg);  // MAG
		param2 = mavlink_msg_command_long_get_param5(msg); // ACC
		if (param) {
		    //supervisorCalibrate();
		    ack = MAV_CMD_ACK_OK;
		}
		if (param2) {
#ifdef HAS_DIGITAL_IMU
		    supervisorTare();
		    ack = MAV_CMD_ACK_OK;
#else
		    ack = MAV_CMD_ACK_ERR_FAIL;
		    AQ_NOTICE("Error: Can't perform Tare, no Digital IMU.");
#endif
		}
	    }
	    else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Can't calibrate while armed!");
	    }

	    break;  // case MAV_CMD_PREFLIGHT_CALIBRATION

	case MAV_CMD_PREFLIGHT_STORAGE:
	    if (!(supervisorData.state & STATE_FLYING)) {
		param = mavlink_msg_command_long_get_param1(msg);
		switch (compId) {

		    // IMU calibration parameters
		    case MAV_COMP_ID_IMU:
#if defined(HAS_DIGITAL_IMU) && defined(DIMU_HAVE_EEPROM)
			if (param == 0.0f) {
			    dIMURequestCalibRead();
			    ack = MAV_CMD_ACK_OK;
			}
			else if (param == 1.0f) {
			    dIMURequestCalibWrite();
			    ack = MAV_CMD_ACK_OK;
			}
#else
			ack = MAV_CMD_ACK_ERR_FAIL;
			AQ_NOTICE("Error: No Digital IMU with EEPROM is available.");
#endif

			break; // case MAV_COMP_ID_IMU

		    // main parameters
		    default:
			if (param == 0.0f) {  					// read flash
			    configFlashRead();
			    ack = MAV_CMD_ACK_OK;
			}
			else if (param == 1.0f && configFlashWrite()) 		// write flash
			    ack = MAV_CMD_ACK_OK;
			else if (param == 4.0f) {				// load defaults
			    configLoadDefault();
			    ack = MAV_CMD_ACK_OK;
			}
			else
			    ack = MAV_CMD_ACK_ERR_FAIL;

			break; // case default (main params)

		} // switch(component ID)
	    }
	    else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Can't save or load parameters while flying!");
	    }

	    break; // case MAV_CMD_PREFLIGHT_STORAGE

	

	// enable/disable AQ custom dataset messages and set frequency
	case MAV_CMD_AQ_TELEMETRY:
            param = mavlink_msg_command_long_get_param1(msg);	// start/stop
            param2 = mavlink_msg_command_long_get_param2(msg);	// interval in us
            param3 = mavlink_msg_command_long_get_param3(msg);	// dataset id

            if (param3 < AQMAV_DATASET_ENUM_END) {
        	enable = (uint8_t)param;
		mavlinkData.customDatasets[(uint8_t)param3] = enable;
		// "legacy" diagnostic telemetry mode
		if ((uint8_t)param3 == AQMAV_DATASET_LEGACY1) {
		    mavlinkData.customDatasets[AQMAV_DATASET_LEGACY2] = enable;
		    mavlinkData.customDatasets[AQMAV_DATASET_LEGACY3] = enable;
		    // toggle all other streams because legacy mode sends a lot of data
		    mavlinkToggleStreams(!enable);
		}
		// check if any datasets are active and enable/disable EXTRA3 stream accordingly
		// AQMAV_DATASET_ALL is special and toggles all datasets
		if (!enable && (uint8_t)param3 != AQMAV_DATASET_ALL) {
		    for (i=0; i < AQMAV_DATASET_ENUM_END; ++i)
			if (mavlinkData.customDatasets[i]) {
			    enable = 1;
			    break;
			}
		}
		mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].enable = enable;
		// note that specified interval currently affects all custom datasets in the EXTRA3 stream
		if (param2)
		    mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].interval = (unsigned long)param2;

		ack = MAV_CMD_ACK_OK;
            } else {
        	ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Unknown telemetry dataset requested.");
            }

            break; // case MAV_CMD_AQ_TELEMETRY

	// send firmware version number;
	case MAV_CMD_AQ_REQUEST_VERSION:
	    utilVersionString();
	    utilSerialNoString();
#ifdef USE_QUATOS
	    AQ_NOTICE("Quatos enabled.");
#endif
	    ack = MAV_CMD_ACK_OK;
	    break;

	default:
	    AQ_PRINTF("Error: Unrecognized command: %u", command);
	    break;

    } // switch(command)

    mavlink_msg_command_ack_send(MAVLINK_COMM_0, command, ack);
}

void mavlinkRecvTaskCode(commRcvrStruct_t *r) {
    mavlink_message_t msg;
    char paramId[17];
    uint8_t c;

    // process incoming data
    while (commAvailable(r)) {
	c = commReadChar(r);
	// Try to get a new message
	if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlinkData.mavlinkStatus)) {
	    // Handle message
	    switch(msg.msgid) {
		// TODO: finish this block
		case MAVLINK_MSG_ID_COMMAND_LONG:
		    if (mavlink_msg_command_long_get_target_system(&msg) == mavlink_system.sysid) {
			mavlinkDoCommand(&msg);
		    }
		    break;

		case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
		    if (mavlink_msg_change_operator_control_get_target_system(&msg) == mavlink_system.sysid) {
			mavlink_msg_change_operator_control_ack_send(MAVLINK_COMM_0, msg.sysid, mavlink_msg_change_operator_control_get_control_request(&msg), 0);
		    }
		    break;

	


		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		    if (mavlink_msg_param_request_read_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t paramIndex;

			paramIndex = mavlink_msg_param_request_read_get_param_index(&msg);
			if (paramIndex < CONFIG_NUM_PARAMS)
			    mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameterStrings[paramIndex], p[paramIndex], MAVLINK_TYPE_FLOAT, CONFIG_NUM_PARAMS, paramIndex);
		    }
		    break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		    if (mavlink_msg_param_request_list_get_target_system(&msg) == mavlink_system.sysid) {
			mavlinkData.currentParam = 0;
			mavlinkData.nextParam = 0;
		    }
		    break;

		case MAVLINK_MSG_ID_PARAM_SET:
		    if (mavlink_msg_param_set_get_target_system(&msg) == mavlink_system.sysid) {
			int paramIndex = -1;
			float paramValue;

			mavlink_msg_param_set_get_param_id(&msg, paramId);
			paramIndex = configGetParamIdByName((char *)paramId);
			if (paramIndex >= 0 && paramIndex < CONFIG_NUM_PARAMS) {
			    paramValue = mavlink_msg_param_set_get_param_value(&msg);
			    if (!isnan(paramValue) && !isinf(paramValue) && !(supervisorData.state & STATE_FLYING))
				p[paramIndex] = paramValue;
			    // send back what we have no matter what
			    mavlink_msg_param_value_send(MAVLINK_COMM_0, paramId, p[paramIndex], MAVLINK_TYPE_FLOAT, CONFIG_NUM_PARAMS, paramIndex);
			}
		    }
		    break;

		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
		    if (mavlink_msg_request_data_stream_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t rate;
			uint8_t stream_id, enable;

			stream_id = mavlink_msg_request_data_stream_get_req_stream_id(&msg);
			rate = mavlink_msg_request_data_stream_get_req_message_rate(&msg);
			rate = constrainInt(rate, 0, 200);
			enable = mavlink_msg_request_data_stream_get_start_stop(&msg);

			// STREAM_ALL is special:
			// to disable all streams entirely (except heartbeat), set STREAM_ALL start = 0
			// to enable literally all streams at a certain rate, set STREAM_ALL rate > 0 and start = 1;
			// set rate = 0 and start = 1 to disable sending all data and return back to your regularly scheduled streams :)
			if (stream_id == MAV_DATA_STREAM_ALL)
			    mavlinkToggleStreams(enable);

			if (stream_id < AQMAVLINK_TOTAL_STREAMS) {
			    mavlinkData.streams[stream_id].enable = rate && enable;
			    mavlinkData.streams[stream_id].interval = rate ? 1e6 / rate : 0;
			    mavlink_msg_data_stream_send(MAVLINK_COMM_0, stream_id, rate, mavlinkData.streams[stream_id].enable);
			}
		    }
		    break;

		case MAVLINK_MSG_ID_OPTICAL_FLOW:
		    navUkfOpticalFlow(mavlink_msg_optical_flow_get_flow_x(&msg),
					mavlink_msg_optical_flow_get_flow_y(&msg),
					mavlink_msg_optical_flow_get_quality(&msg),
					mavlink_msg_optical_flow_get_ground_distance(&msg));
		    break;

		case MAVLINK_MSG_ID_HEARTBEAT:
		    break;

		default:
		    // Do nothing
		    break;
	    }
	}

	// Update global packet drops counter
	mavlinkData.packetDrops += mavlinkData.mavlinkStatus.packet_rx_drop_count;
    }
}



void mavlinkSendParameter(uint8_t sysId, uint8_t compId, const char *paramName, float value) {
    mavlink_msg_param_set_send(MAVLINK_COMM_0, sysId, compId, paramName, value, MAV_PARAM_TYPE_REAL32);
}

void mavlinkInit(void) {
    unsigned long micros, hz;
    int i;

    memset((void *)&mavlinkData, 0, sizeof(mavlinkData));

    // register notice function with comm module
    commRegisterNoticeFunc(mavlinkSendNotice);
    commRegisterTelemFunc(mavlinkDo);
    commRegisterRcvrFunc(COMM_STREAM_TYPE_MAVLINK, mavlinkRecvTaskCode);

    AQ_NOTICE("Mavlink init\n");

    mavlinkData.currentParam = CONFIG_NUM_PARAMS;
    mavlink_system.sysid = flashSerno(0) % 250;
    mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;

    mavlinkData.streams[MAV_DATA_STREAM_ALL].dfltInterval = AQMAVLINK_STREAM_RATE_ALL;
    mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].dfltInterval = AQMAVLINK_STREAM_RATE_RAW_SENSORS;
    mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].dfltInterval = AQMAVLINK_STREAM_RATE_EXTENDED_STATUS;
    mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].dfltInterval = AQMAVLINK_STREAM_RATE_RC_CHANNELS;
    mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].dfltInterval = AQMAVLINK_STREAM_RATE_RAW_CONTROLLER;
    mavlinkData.streams[MAV_DATA_STREAM_POSITION].dfltInterval = AQMAVLINK_STREAM_RATE_POSITION;
    mavlinkData.streams[MAV_DATA_STREAM_EXTRA1].dfltInterval = AQMAVLINK_STREAM_RATE_EXTRA1;
    mavlinkData.streams[MAV_DATA_STREAM_EXTRA2].dfltInterval = AQMAVLINK_STREAM_RATE_EXTRA2;
    mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].dfltInterval = AQMAVLINK_STREAM_RATE_EXTRA3;

    // turn on streams & spread them out
    micros = timerMicros();
    for (i = 0; i < AQMAVLINK_TOTAL_STREAMS; i++) {
	mavlinkData.streams[i].interval = mavlinkData.streams[i].dfltInterval;
	mavlinkData.streams[i].next = micros + 5e6f + i * 5e3f;
	mavlinkData.streams[i].enable = mavlinkData.streams[i].interval ? 1 : 0;
	hz = mavlinkData.streams[i].interval ? 1e6 / mavlinkData.streams[i].interval : 0;
	mavlink_msg_data_stream_send(MAVLINK_COMM_0, i, hz, mavlinkData.streams[i].enable);
    }
}
#endif	// USE_MAVLINK
