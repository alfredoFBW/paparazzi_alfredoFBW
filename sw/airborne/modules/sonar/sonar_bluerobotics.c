/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/*
 * @file "modules/sensors/sonar/sonar_bluerobotics.c"
 * @authors Jesús Bautista Villar
 *          Juan Francisco Jiménez Castellanos
 *          Lía García Pérez
 *          Hector Garcia de Marina
 * 
 */

#include "modules/sonar/sonar_bluerobotics.h"

#include "generated/airframe.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"

#include "std.h"
#include <stdio.h>

struct sonar_parse_t br_sonar;

bool sonar_stream_setting;

// Sonar msg header bytes (and checksum)
static uint8_t headerLength = 8;
static uint8_t checksumLength = 2;

static uint8_t SONAR_START1_BYTE = 0x42; // "B"
static uint8_t SONAR_START2_BYTE = 0x52; // "R"


// Sonar parse states
#define BR_INIT 0
#define BR_SYNC1 1
#define BR_PAYLOAD_LEN1 2
#define BR_PAYLOAD_LEN2 3
#define BR_MSG_ID1 4
#define BR_MSG_ID2 5
#define BR_SRC_ID 6
#define BR_DEV_ID 7
#define BR_PAYLOAD1 8
#define BR_CHECKSUM1 9
#define BR_CHECKSUM2 10

// Sonar errors
/* last error type */
#define SONAR_BR_ERR_NONE         0
#define SONAR_BR_ERR_OVERRUN      1
#define SONAR_BR_ERR_MSG_TOO_LONG 2
#define SONAR_BR_ERR_CHECKSUM     3
#define SONAR_BR_ERR_UNEXPECTED   4
#define SONAR_BR_ERR_OUT_OF_SYNC  5


// Messages sent
#define BR_RQ_SONAR_START 0
#define BR_RQ_SONAR_STOP 1
#define BR_RQ_SOUND_VEL 2
#define BR_RQ_FIRMWARE 3
#define BR_RQ_DISTANCE_SIMPLE 4
#define BR_RQ_DEVICE_ID 5

// Testing variable
uint8_t checksum_test;
uint8_t byte_leido;
bool primera_vez = true;

uint8_t modo=BR_RQ_SONAR_START;

/* TODO: Revisar que estos mensajes son correctos y que el sonar responde a ellos */
// Test request message: (general_request#6 --> common_protocol_version#5)
static uint8_t request_protocol_version[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x06, //  4: 6_L message ID |
        0x00, //  5: 0_H            |
        0x00, //  6: source ID
        0x00, //  7: device ID
        0x05, //  8: 5_L requested message ID |
        0x00, //  9: 0_H                      |
        0xA1, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x00  // 11: 0_H                                                    |
};

// Test request message: (general_request#6 --> distance_simple#1211)
static uint8_t request_simple_distance[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x06, //  4: 6_L message ID |
        0x00, //  5: 0_H            |
        0x00, //  6: source ID
        0x01, //  7: device ID
        0xBB, //  8: BB_L requested message ID |
        0x04, //  9: 4_H                      |
        0x5C, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x01  // 11: 0_H                                                    |
};

static uint8_t request_simple_distance_streaming[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x78, //  4: 6_L message ID |
        0x05, //  5: 0_H            |
        0x00, //  6: source ID
        0x01, //  7: device ID
        0xBB, //  8: BB_L requested message ID |
        0x04, //  9: 4_H                      |
        0xD3, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x01  // 11: 0_H                                                    |
};

static uint8_t request_simple_distance_streaming_stop[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x79, //  4: 6_L message ID |
        0x05, //  5: 0_H            |
        0x00, //  6: source ID
        0x01, //  7: device ID
        0xBB, //  8: BB_L requested message ID |
        0x04, //  9: 4_H                      |
        0xD4, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x01  // 11: 0_H                                                    |
};
static uint8_t request_speed_sound[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x06, //  4: 6_L message ID |
        0x00, //  5: 0_H            |
        0x00, //  6: source ID
        0x01, //  7: device ID
        0xB3, //  8: BB_L requested message ID |
        0x04, //  9: 4_H                      |
        0x54, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x01  // 11: 0_H                                                    |
};

static uint8_t request_device_id[12] = { 
        0x42, //  0: "B"
        0x52, //  1: "R"
        0x02, //  2: 2_L payload length |
        0x00, //  3: 0_H                |
        0x06, //  4: 6_L message ID |
        0x00, //  5: 0_H            |
        0x00, //  6: source ID
        0x01, //  7: device ID
        0xB1, //  8: BB_L requested message ID |
        0x04, //  9: 4_H                      |
        0x52, // 10: 161_L message checksum (sum of all non-checksum bytes) |
        0x01  // 11: 0_H                                                    |
};

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/* Telemetry functions (TESTING TELEMETRY) */
static void send_telemetry(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_BR_SONAR(trans, dev, AC_ID,	
  						&br_sonar.protocol_version,
  						&br_sonar.protocol_subversion,
  						&br_sonar.protocol_patch,
  						&br_sonar.payload_len,
  						&br_sonar.msg_id,
  						&br_sonar.src_id,
  						&br_sonar.dev_id,
  						&br_sonar.status,
  						&br_sonar.error_last,
  						&br_sonar.ck,
  						&br_sonar.distance,
  						&br_sonar.confidence,
  						&br_sonar.sound_vel);
}
#endif


/* Initialize decoder */
void sonar_init(void)
{ 
  br_sonar.status = BR_INIT;
  br_sonar.msg_available = true;

  sonar_stream_setting = true;
  checksum_test = 0;
  
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_BR_SONAR, send_telemetry);
  #endif

}


/* Send message to serial port (byte by byte) */
static void sonar_send_msg(uint8_t len, uint8_t *bytes)
{
  struct link_device *dev = &((SONAR_DEV).device);

  int i = 0;
  for (i = 0; i < len; i++) {
    dev->put_byte(dev->periph, 0, bytes[i]);
  }

  br_sonar.msg_available = false;
}


/* Message parsing functions */
static uint32_t msgLength(void){
  return headerLength + br_sonar.payload_len + checksumLength;
};

static uint32_t calculateChecksum(void){
  uint32_t i = 0;
  uint32_t non_ck_len = msgLength() - checksumLength;
  br_sonar.ck = 0;

  for(i = 0; i < non_ck_len; i++) {
    br_sonar.ck += br_sonar.msgData[i];
  }

  return br_sonar.ck;
};

unsigned int byteToint(uint8_t * bytes,int length){
    unsigned int num=0 ;
    for (int i=length-1;i>=0;i--){
        num=num|bytes[i]<<(8*(i));
   }
    return num;
}

static void sonar_parse_simple_distance(void){
    //Confidence
	br_sonar.confidence=(int)br_sonar.msgData[12];
        uint8_t distanceBytes[4]={
        br_sonar.msgData[8],
        br_sonar.msgData[9],
        br_sonar.msgData[10],
        br_sonar.msgData[11]
        };
        br_sonar.distance=byteToint(distanceBytes,4);
  }

static void sonar_parse_firmware(void){
	br_sonar.protocol_version = (int)br_sonar.msgData[8];
	br_sonar.protocol_subversion = (int) br_sonar.msgData[9];
	br_sonar.protocol_patch = (int) br_sonar.msgData[10];
}

static void sonar_parse_sound_speed(void){
	uint8_t sound_vel[4]={
  				br_sonar.msgData[8],
        			br_sonar.msgData[9],
        			br_sonar.msgData[10],
        			br_sonar.msgData[11]
        };
        br_sonar.sound_vel=byteToint(sound_vel,4);
   
}

static void sonar_parse_device_id(void){
	br_sonar.dev_id = (int) br_sonar.msgData[8];
}


/* Sonar message parser */

void sonar_read_message(void){

// Checksum
	uint8_t chksBytes[2]={br_sonar.msgData[br_sonar.payload_len+8],br_sonar.msgData[br_sonar.payload_len+9]};
	unsigned int chks=byteToint(chksBytes,2);
	if(chks!=(unsigned int)calculateChecksum()){
		br_sonar.error_last=SONAR_BR_ERR_CHECKSUM;
		}
 	switch (br_sonar.msg_id){
 		case BR_DISTANCE_SIMPLE:
 			sonar_parse_simple_distance();
 			break;
 		case BR_FIRMWARE:
 			sonar_parse_firmware();
 			break;
 		case BR_SOUND_SPEED:
 			sonar_parse_sound_speed();
 			break;
 		case BR_DEVICE_ID:
 			sonar_parse_device_id();
 			break;
 		default:
 			br_sonar.error_last = SONAR_BR_ERR_UNEXPECTED;
 			break;
 		};
}


static void sonar_parse(uint8_t byte){
	bool error=false;
	bool restart = false;
	static int payload_count;
	switch (br_sonar.status){
		
		case BR_INIT: // First byte of a new message
			if (byte == SONAR_START1_BYTE) { //Has to be an 'B'
				memset(br_sonar.msgData,0,16);
				br_sonar.status++;
				br_sonar.msgData[0]=byte;
				br_sonar.msg_available=false; 
				}
			break;
		case BR_SYNC1: //Second byte
			
			if (byte != SONAR_START2_BYTE) { //Has to be an 'R'
				br_sonar.error_last = SONAR_BR_ERR_OUT_OF_SYNC;
				error=true;
				}
			br_sonar.status++;
			br_sonar.msgData[1]=byte;
			
			break;
		case BR_PAYLOAD_LEN1: //3th byte pyload length byte 1
			br_sonar.payload_len = (int) byte;
			br_sonar.status++;
			br_sonar.msg_id = 0;
			br_sonar.msgData[2]=byte;
			
			break;
		case BR_PAYLOAD_LEN2: //4th byte payload length byte 2
			br_sonar.status++;
			br_sonar.msgData[3]=byte;
			
			break;
		case BR_MSG_ID1: //5th byte first byte messg ID
			br_sonar.status++;
			br_sonar.msgData[4]=byte;
			
			break;
		case BR_MSG_ID2: //6th byte first byte messg ID
			br_sonar.status++;
			br_sonar.msgData[5]=byte;
			uint8_t id_bytes[2];
			id_bytes[0] = br_sonar.msgData[4];
			id_bytes[1] = br_sonar.msgData[5];
			br_sonar.msg_id=byteToint(id_bytes,2);
			break;
		case BR_SRC_ID:
			br_sonar.status++;
			br_sonar.msgData[6]=byte;
			
			break;
		case BR_DEV_ID:
			br_sonar.status++;
			br_sonar.msgData[7]=byte;
			payload_count= 0;
			break;

		case BR_PAYLOAD1:
			payload_count++;
			if(payload_count<(br_sonar.payload_len)){
				br_sonar.msgData[7+payload_count]=byte;
				}
			else{
			 	br_sonar.msgData[7+payload_count]=byte;
			 	br_sonar.status++;
			 	payload_count=0;
			 	}
			
			break;
		case BR_CHECKSUM1:
			br_sonar.status++;
			br_sonar.msgData[br_sonar.payload_len+8]=byte;
			break;
		case BR_CHECKSUM2:
			br_sonar.status++;
			br_sonar.msgData[br_sonar.payload_len+9]=byte;
			br_sonar.msg_available = true;
			br_sonar.status=0;
			restart=true;
			break;	
		default:
			br_sonar.error_last = SONAR_BR_ERR_UNEXPECTED;
			br_sonar.status= 0;
			error=true;
	};
	if(error){
		br_sonar.error_cnt++;
		br_sonar.status=0;
		return;
		}
	if (restart){
		br_sonar.status =  BR_INIT;
		return;
		}
	return;

}; 


void sonar_event(void)
{
 
 while(uart_char_available(&(SONAR_DEV))){
 	uint8_t ch= uart_getch(&(SONAR_DEV));
 	sonar_parse(ch);

 	if (br_sonar.msg_available) {
      		sonar_read_message();

         
       	}	
    }
}

// Send ping message
void sonar_ping(void)
{   

switch (modo) {
	case BR_RQ_SONAR_START:
		if(br_sonar.msg_available && primera_vez){
	  		sonar_send_msg(12,request_simple_distance_streaming);
	  		primera_vez=false;
	  	}
		break;
	case BR_RQ_SONAR_STOP:
		if(br_sonar.msg_available && !primera_vez){
	  		sonar_send_msg(12,request_simple_distance_streaming_stop);
	  	}
		break;
	case BR_RQ_SOUND_VEL:
		if(br_sonar.msg_available){
	  		sonar_send_msg(12,request_speed_sound);
	  	}
		break;
	case BR_RQ_FIRMWARE:
		if(br_sonar.msg_available){
	  		sonar_send_msg(12,request_protocol_version);

	  	}
		break;
	case BR_RQ_DISTANCE_SIMPLE:
		if(br_sonar.msg_available){
	  		sonar_send_msg(12,request_simple_distance);
	  	}
		break;
	case BR_RQ_DEVICE_ID:
		if(br_sonar.msg_available){
	  		sonar_send_msg(12,request_device_id);
	  	}
		break;
	default:
		break;
		}
}

//Get function
struct sonar_parse_t * sonar_get(void){
	return(&br_sonar);

}

