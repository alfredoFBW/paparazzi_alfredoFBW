/*
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
 
#ifndef SONAR_BLUEROBOTICS_H
#define SONAR_BLUEROBOTICS_H

#include "std.h"

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

/* Parser variables */
/* TODO: Tener una pequeña librería en el .h con todas las IDs que vamos a usar.
         Lo ideal sería construir un parser que fuera capaz de leer cualquier mensaje.
         Se puede diseñar el parse de forma general como una máquina de estados, así lo
         hacen en el módulo de GPS de ublox. Tenemos mucho código de refrencia si pocedemos
         de esto modo, pienso que nos será bastante más sencillo. */

// Common IDs
#define BR_PROTOCOL_VERSION 5
#define BR_REQUEST 6

// Ping1D IDs
#define BR_SET_DEVICE_ID 1000
#define BR_DEVICE_ID 1201
#define BR_DISTANCE_SIMPLE 1211
#define BR_FIRMWARE 5
#define BR_SOUND_SPEED 1203

// Variable to start/stop requesting stream
extern bool sonar_stream_setting;

/* Testing variables */
extern uint8_t checksum; // Telemetry testing variable

/* Parser msg struct */
#define SONAR_MAX_PAYLOAD 256

struct sonar_parse_t {
  uint16_t payload_len;
  uint16_t msg_id;
  uint8_t src_id;
  uint8_t dev_id;

  uint8_t msgData[SONAR_MAX_PAYLOAD] __attribute__((aligned));
  uint8_t status;
  
  uint8_t error_cnt;
  uint8_t error_last;
  
  uint16_t ck;
  bool msg_available;
  
  uint8_t protocol_version;
  uint8_t protocol_subversion;
  uint8_t protocol_patch;
  
  uint32_t sound_vel;
  uint32_t distance;
  uint8_t confidence;
};

void sonar_read_message(void);

unsigned int byteToint(uint8_t * bytes,int length);



extern struct sonar_parse_t br_sonar;

	
/* External functions (called by the autopilot)*/
extern void sonar_init(void);
extern void sonar_ping(void);
extern void sonar_event(void);
extern struct sonar_parse_t * sonar_get(void);

#endif //SONAR_BLUEROBOTICS_H
