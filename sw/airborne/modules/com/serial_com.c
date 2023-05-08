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

#include "modules/com/serial_com.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"
#include "modules/sonar/sonar_bluerobotics.h"

#include "std.h"
#include <stdio.h>


struct serial_parse_t serial_msg;
struct serial_send_t serial_snd;

bool serial_msg_setting;

// Sonar msg header bytes (and checksum)
static uint8_t headerLength = 2;
static uint8_t checksumLength = 2;

static uint8_t PPZ_START_BYTE = 0x50; // "P"
static uint8_t COM_START_BYTE = 0x52; // "R"
static uint8_t PPZ_SONAR_BYTE = 0x53; // "S"
static uint8_t PPZ_TELEMETRY_BYTE = 0x54; // "T"
static uint8_t PPZ_MEASURE_BYTE = 0x4D; // "M"

static uint32_t last_s = 0;  // timestamp in usec when last message was send
#define SEND_INTERVAL 1000 // time between sending messages
// Sonar parse states
#define SR_INIT 0
#define SR_SYNC 1
#define SR_PAYLOAD1 2
#define SR_PAYLOAD2 3
#define SR_PAYLOAD3 4
#define SR_PAYLOAD4 5
#define SR_CHECKSUM1 6
#define SR_CHECKSUM2 7
// Sonar errors
/* last error type */
#define SERIAL_BR_ERR_NONE         0
#define SERIAL_BR_ERR_CHECKSUM    1
#define SERIAL_ERR_UNEXPECTED   2



// Messages sent
#define TELEMETRY_SN 0
#define SONDA_RQ 1
#define MEASURE_SN 2

//Messages received
#define SR_OK 79
#define SR_MEASURE 77
#define SR_FIN 70
#define SR_POS 80
#define SR_ERR_L 76
#define SR_ERR_D 78

//Measuring mode
#define BR_SONAR_MS_1 1
#define BR_SONAR_MS_0 0

uint8_t message_type=TELEMETRY_SN;
/*Quitar*/
int cont=0;
/*-------------------*/

int modo_medida=BR_SONAR_MS_0;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/* Telemetry functions (TESTING TELEMETRY) */

static void send_telemetry(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_SERIAL_COM(trans, dev, AC_ID,	&serial_snd.time,
  						&serial_snd.lat,
  						&serial_snd.lon,
  						&serial_snd.alt,
  						&serial_snd.distance,
  						&serial_snd.confidence,
  						&serial_snd.error_last,
  						&serial_snd.msg_length,
  						&serial_snd.ck,
  						&serial_msg.msg_id,
  						&serial_msg.status,
  						&serial_msg.payload_len,
  						&serial_msg.time,
  						&serial_msg.depth,
  						&serial_msg.error_last,
  						&serial_msg.ck
  						);
}
#endif


/* Initialize decoder */
void serial_init(void)
{ 
  serial_msg.status = SR_INIT;
  serial_msg.msg_available = true;
  serial_msg.error_last=0;
  serial_msg_setting = true;
  
  last_s=get_sys_time_msec();
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SERIAL_COM, send_telemetry);
  #endif

}

/* Message parsing functions */
static uint32_t msgLength(void){
  return headerLength + serial_msg.payload_len + checksumLength;
};

/* Send message to serial port (byte by byte) */
static void serial_send_msg(uint8_t len, uint8_t *bytes)
{
  struct link_device *dev = &((SERIAL_DEV).device);
	
  int i = 0;

  for (i = 0; i < len; i++) {
    dev->put_byte(dev->periph, 0, bytes[i]);

  }


}


static uint32_t serial_calculateChecksum(void){
  uint32_t i = 0;
  uint32_t non_ck_len = msgLength() - checksumLength;
  serial_msg.ck = 0;

  for(i = 0; i < non_ck_len; i++) {
    serial_msg.ck += serial_msg.msgData[i];
  }

  return serial_msg.ck;
}

void serial_calculateChecksumMsg(uint8_t *msg, int msgLength){
  int i = 0;
  uint16_t suma=0;
  int non_ck_len = msgLength - checksumLength;
  
  for(i = 0; i < non_ck_len; i++) {
    suma += msg[i];
  }
  serial_snd.ck=suma;
  uint8_t chksBytes[2];
  ito2h(suma,chksBytes);
  msg[msgLength-2]=chksBytes[1];
  msg[msgLength-1]=chksBytes[0];


};


unsigned int serial_byteToint(uint8_t * bytes,int length){
    unsigned int num=0 ;
    for (int i=length-1;i>=0;i--){
        num=num|bytes[i]<<(8*(i));
   }
    return num;
}

void ito2h(int value, unsigned char* str) {
	
    str[1]=(value & 0xff00)>>8;
    str[0]= value & 0x0ff;	
}

void itoh(int value, unsigned char* str, int nbytes){
    double nmax;
    int nb;
    nmax=pow(2,(nbytes-1)*8)-1;
    nb=nbytes;
    if (abs(value) > nmax ) return;
    if (value <0) {
       str[0]=1;
       value=-value;
       }
    else str[0]=0;
    for (int i=1; i<nb; i++){
           str[i]=(value & (0xff << (nb-1-i)*8) ) >> (8*(nb-1-i));
        }

    return;

}


static void message_parse(void){
	uint8_t msgBytes[2]={serial_msg.msgData[3],serial_msg.msgData[2]};
	serial_msg.time = serial_byteToint(msgBytes,2);
	memset(msgBytes,0,2);
	msgBytes[0]=serial_msg.msgData[5];
	msgBytes[1]=serial_msg.msgData[4];
	serial_msg.depth=serial_byteToint(msgBytes,2);
  }
  
  

static void message_OK_parse(void){
	uint8_t msgBytes[2]={serial_msg.msgData[3],serial_msg.msgData[2]};
	serial_msg.time = serial_byteToint(msgBytes,2);

  }
  
  
/* Serial message parser */

void serial_read_message(void){

// Checksum
	uint8_t chksBytes[2];
	if (serial_msg.msg_id == SR_OK){
		chksBytes[0]=serial_msg.msgData[5];
		chksBytes[1]=serial_msg.msgData[4];
		}
	else {
		chksBytes[0]=serial_msg.msgData[7];
		chksBytes[1]=serial_msg.msgData[6];

	}
	uint16_t chks;
	chks=serial_byteToint(chksBytes,2);
	serial_calculateChecksum();
	switch (serial_msg.msg_id){
 		case SR_OK:
 			message_OK_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
 			message_type = 10;
 			break;
 		case SR_FIN:
 			message_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
			message_type = TELEMETRY_SN;
			modo_medida = BR_SONAR_MS_0;
 			break;
 		case SR_MEASURE:
 			message_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
			message_type = 10;
 			break;
 		case SR_POS:
 			message_parse();
 			serial_msg.error_last = SERIAL_BR_ERR_NONE;
 			message_type = 10;
 			break;
 		case SR_ERR_L:
 			serial_msg.error_last = SR_ERR_L;
 			break;
 		case SR_ERR_D:
 			serial_msg.error_last = SR_ERR_D;
 			break;
 		default:
 			serial_msg.error_last = SERIAL_ERR_UNEXPECTED;
 			break;
 		};
 	if(chks!=(unsigned int) serial_msg.ck){
		serial_msg.error_last=SERIAL_BR_ERR_CHECKSUM;
		serial_msg.error_cnt++;
		}
 
}


static void serial_parse(uint8_t byte){
	bool error=false;
	bool restart = false;

	switch (serial_msg.status){
		
		case SR_INIT: // First byte of a new message
			
			if (byte == COM_START_BYTE) { //Has to be an 'R'
				memset(serial_msg.msgData,0,8);
				serial_msg.status++;
				serial_msg.msgData[0]=byte;
				serial_msg.msg_available=false; 
				}
			break;
		case SR_SYNC: //Second byte
			
			serial_msg.status++;
			serial_msg.msg_id = byte;
			if(byte == SR_OK) serial_msg.payload_len=2;
			else serial_msg.payload_len=4;			
			serial_msg.msgData[1]=byte;
			break;
		case SR_PAYLOAD1: //3th byte 
			serial_msg.status++;
			serial_msg.msgData[2]=byte;		
			break;
		case SR_PAYLOAD2: //4th byte payload length byte 2
			if(serial_msg.msg_id==SR_OK){
				serial_msg.status +=2;
				
				}
			serial_msg.status++;
			serial_msg.msgData[3]=byte;
			break;
		case SR_PAYLOAD3:
			serial_msg.error_last = 0;
			serial_msg.status++;
			serial_msg.msgData[4]=byte;
			break;
		case SR_PAYLOAD4:
			serial_msg.error_last = 0;
			serial_msg.status++;
			serial_msg.msgData[5]=byte;
			break;
		case SR_CHECKSUM1:
			serial_msg.error_last = 0;
			serial_msg.status++;
			if (serial_msg.msg_id==SR_OK)
				serial_msg.msgData[4]=byte;
			else
				serial_msg.msgData[6]=byte;
				
			break;
		case SR_CHECKSUM2:
			serial_msg.error_last = 0;
			serial_msg.status++;
			if (serial_msg.msg_id==SR_OK)
				serial_msg.msgData[5]=byte;
			else 
				serial_msg.msgData[7]=byte;
			serial_msg.msg_available = true;
			serial_msg.status=0;
			restart=true;
			break;	
		default:
			serial_msg.error_last = SERIAL_ERR_UNEXPECTED;
			serial_msg.status= 0;
			error=true;
	};
	if(error){
		serial_msg.error_cnt++;
		serial_msg.status=0;
		serial_msg.msg_available = true;
		return;
		}
	if (restart){
		serial_msg.status =  SR_INIT;
		return;
		}
	return;

}; 


void serial_event(void)
{
 
 while(uart_char_available(&(SERIAL_DEV))){
 	uint8_t ch= uart_getch(&(SERIAL_DEV));
	
 	serial_parse(ch);
	
 	if (serial_msg.msg_available) {
      		serial_read_message();

         
       	}	
    }
}

// Send ping message
void serial_ping(void)
{   
 uint32_t now_s = get_sys_time_msec();
 /*
struct LlaCoor_i {
  int32_t lat; ///< in degrees*1e7
  int32_t lon; ///< in degrees*1e7
  int32_t alt; ///< in millimeters above WGS84 reference ellipsoid
};
 */
struct LlaCoor_i *gps_coord;
struct sonar_parse_t *sonar_data;
uint8_t msg_gps[5]={0,0,0,0,0};
uint8_t msg_time[2]={0,0};
uint8_t msg_dist[5]={0,0,0,0,0};
if (now_s > (last_s+ SEND_INTERVAL)) {
    	last_s = now_s; 

	if(serial_msg.msg_available){
		serial_read_message();
		switch(message_type){
			case SONDA_RQ:
				serial_snd.msg_length=6;
				
				serial_snd.msgData[0]=PPZ_START_BYTE;
				serial_snd.msgData[1]=PPZ_SONAR_BYTE;
				serial_snd.time=sys_time.nb_sec;
				message_type=TELEMETRY_SN;
				ito2h(serial_snd.time, msg_time);
				serial_snd.msgData[2]=msg_time[0];
				serial_snd.msgData[3]=msg_time[1];
				serial_calculateChecksumMsg(serial_snd.msgData, (int)serial_snd.msg_length);
				serial_send_msg(serial_snd.msg_length,serial_snd.msgData); 
				
				break;
			case TELEMETRY_SN:
				serial_snd.msg_length=25;
				memset(serial_snd.msgData,0,25);
				serial_snd.msgData[0]=PPZ_START_BYTE;
				serial_snd.msgData[1]=PPZ_TELEMETRY_BYTE;
				serial_snd.time=sys_time.nb_sec;
				
				ito2h(serial_snd.time, msg_time);
				serial_snd.msgData[2]=msg_time[0];
				serial_snd.msgData[3]=msg_time[1];
				// Get Position
				
				gps_coord = stateGetPositionLla_i();
				serial_snd.lon=gps_coord->lon;
				serial_snd.lat=gps_coord->lat;
				serial_snd.alt=gps_coord->alt;
				itoh(gps_coord->lon,msg_gps,5);
				for(int i=0;i<5;i++) serial_snd.msgData[i+4]=msg_gps[i];
				memset(msg_gps,0,5);
				itoh(gps_coord->lat,msg_gps,5);
				for(int i=0;i<5;i++) serial_snd.msgData[i+9]=msg_gps[i];
				memset(msg_gps,0,5);
				/*NOTE: serial_snd.alt is an unsigned int. It is codified as 
				an signed (using one extra byte) int but sign byte is discarded
				*/
				itoh(serial_snd.alt,msg_gps,5);
				for(int i=0;i<4;i++) serial_snd.msgData[i+14]=msg_gps[i+1];
				// Get Sonar
				
				sonar_data= sonar_get();
				serial_snd.distance=sonar_data->distance;
				serial_snd.confidence=sonar_data->confidence;
				/*NOTE: serial_snd.distance is an unsigned int. It is codified as 
				an signed (using one extra byte) int but sign byte is discarded
				*/
				
				itoh(sonar_data->distance,msg_dist,5);
				for(int i=0;i<4;i++) serial_snd.msgData[i+18]=msg_dist[i+1];
				serial_snd.msgData[22]=sonar_data->confidence;

				serial_calculateChecksumMsg(serial_snd.msgData, (int)serial_snd.msg_length);
				serial_send_msg(serial_snd.msg_length,serial_snd.msgData); 
				cont++;
				/*QUITAR:Para hacer pruebas*/
				if(cont>=20){
					message_type=MEASURE_SN;
					cont=0;
				}
				/*------------------------------*/
				break;
			case MEASURE_SN:
				message_type=10;
				serial_snd.msg_length=26;
				memset(serial_snd.msgData,0,26);
				serial_snd.msgData[0]=PPZ_START_BYTE;
				serial_snd.msgData[1]=PPZ_MEASURE_BYTE;
				serial_snd.time=sys_time.nb_sec;
				
				ito2h(serial_snd.time, msg_time);
				serial_snd.msgData[2]=msg_time[0];
				serial_snd.msgData[3]=msg_time[1];
				// Get Position
				
				gps_coord = stateGetPositionLla_i();
				serial_snd.lon=gps_coord->lon;
				serial_snd.lat=gps_coord->lat;
				serial_snd.alt=gps_coord->alt;
				itoh(gps_coord->lon,msg_gps,5);
				for(int i=0;i<5;i++) serial_snd.msgData[i+4]=msg_gps[i];
				memset(msg_gps,0,5);
				itoh(gps_coord->lat,msg_gps,5);
				for(int i=0;i<5;i++) serial_snd.msgData[i+9]=msg_gps[i];
				memset(msg_gps,0,5);
				/*NOTE: serial_snd.alt is an unsigned int. It is codified as 
				an signed (using one extra byte) int but sign byte is discarded
				*/
				itoh(serial_snd.alt,msg_gps,5);
				for(int i=0;i<4;i++) serial_snd.msgData[i+14]=msg_gps[i+1];
				// Get Sonar
				
				sonar_data= sonar_get();
				serial_snd.distance=sonar_data->distance;
				serial_snd.confidence=sonar_data->confidence;
				/*NOTE: serial_snd.distance is an unsigned int. It is codified as 
				an signed (using one extra byte) int but sign byte is discarded
				*/
				
				itoh(sonar_data->distance,msg_dist,5);
				for(int i=0;i<4;i++) serial_snd.msgData[i+18]=msg_dist[i+1];
				serial_snd.msgData[22]=sonar_data->confidence;
				serial_snd.msgData[23]=modo_medida;
				serial_calculateChecksumMsg(serial_snd.msgData, (int)serial_snd.msg_length);
				serial_send_msg(serial_snd.msg_length,serial_snd.msgData); 

				break;
				
			default:
				serial_snd.error_last=10 ;
				}

		}
	}
}



