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
 * @file "modules/com/serial_com.c"
 * @authors Jesús Bautista Villar
 *          Juan Francisco Jiménez Castellanos
 *          Lía García Pérez
 *          Hector Garcia de Marina
 *
 */

#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "std.h"

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

/* Parser variables */
// Variable to start/stop requesting stream
extern bool serial_msg_setting;



/* Parser msg struct */
#define SERIAL_MAX_PAYLOAD 26
#define SERIAL_MAX_MSG 8

struct serial_send_t {

  uint8_t msg_length;

  uint8_t msgData[SERIAL_MAX_PAYLOAD];
  uint8_t error_cnt;
  uint8_t error_last;


  uint16_t time;

  int32_t lon;
  int32_t lat;
  int32_t alt;

  uint32_t distance;
  uint8_t confidence;

  uint16_t ck;

};

struct serial_parse_t {

  uint8_t msg_id;

  uint8_t msgData[SERIAL_MAX_MSG] __attribute__((aligned));
  uint8_t status;

  uint8_t error_cnt;
  uint8_t error_last;

  uint8_t payload_len;

  uint16_t ck;
  bool msg_available;


  uint16_t time;
  uint16_t depth;

  // For batteries (voltage and current)
  uint16_t vbat_left;
  uint16_t vbat_right;
  uint16_t cbat_left;
  uint16_t cbat_right;


};

void serial_read_message(void);
void itoh(int value, unsigned char *str, int nbytes);
unsigned int serial_byteToint(uint8_t *bytes, int length);
void ito2h(int value, unsigned char *str) ;
void serial_calculateChecksumMsg(uint8_t *msg, int msgLength);
extern struct serial_parse_t serial_msg;


/* External functions (called by the autopilot)*/
extern void serial_init(void);
extern void serial_ping(void);
extern void serial_event(void);

#endif //SERIAL_COM_H

