/*
* This file is part of smarthomatic, http://www.smarthomatic.org.
* Copyright (c) 2013..2014 Uwe Freese
*
* smarthomatic is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation, either version 3 of the License, or (at your
* option) any later version.
*
* smarthomatic is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with smarthomatic. If not, see <http://www.gnu.org/licenses/>.
*
* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
* ! WARNING: This file is generated by the SHC EEPROM editor and should !
* ! never be modified manually.                                         !
* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/

#ifndef _MSGGRP_POWERSWITCH_H
#define _MSGGRP_POWERSWITCH_H

#include "packet_header.h"
#include "packet_headerext_common.h"
#include "packet_headerext_ackstatus.h"
#include "packet_headerext_ack.h"
#include "packet_headerext_status.h"
#include "packet_headerext_setget.h"
#include "packet_headerext_set.h"
#include "packet_headerext_get.h"
#include "e2p_access.h"

// Message Group "powerswitch"
// ===========================
// MessageGroupID: 20

// ENUM for MessageIDs of this MessageGroup
typedef enum {
  MESSAGEID_POWERSWITCH_SWITCHSTATE = 1,
  MESSAGEID_POWERSWITCH_SWITCHSTATEEXT = 2
} POWERSWITCH_MessageIDEnum;


// Message "powerswitch_switchstate"
// ---------------------------------
// MessageGroupID: 20
// MessageID: 1
// Possible MessageTypes: Get, Set, SetGet, Status, Ack, AckStatus
// Validity: test
// Length w/o Header + HeaderExtension: 17 bits
// Data fields: On, TimeoutSec
// Description: This is the state of the relais and its timeout value.

// Function to initialize header for the MessageType "Get".
static inline void pkg_header_init_powerswitch_switchstate_get(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(0);
  pkg_headerext_get_set_messagegroupid(20);
  pkg_headerext_get_set_messageid(1);
  __HEADEROFFSETBITS = 95;
  __PACKETSIZEBYTES = 16;
  __MESSAGETYPE = 0;
}

// Function to initialize header for the MessageType "Set".
static inline void pkg_header_init_powerswitch_switchstate_set(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(1);
  pkg_headerext_set_set_messagegroupid(20);
  pkg_headerext_set_set_messageid(1);
  __HEADEROFFSETBITS = 95;
  __PACKETSIZEBYTES = 16;
  __MESSAGETYPE = 1;
}

// Function to initialize header for the MessageType "SetGet".
static inline void pkg_header_init_powerswitch_switchstate_setget(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(2);
  pkg_headerext_setget_set_messagegroupid(20);
  pkg_headerext_setget_set_messageid(1);
  __HEADEROFFSETBITS = 95;
  __PACKETSIZEBYTES = 16;
  __MESSAGETYPE = 2;
}

// Function to initialize header for the MessageType "Status".
static inline void pkg_header_init_powerswitch_switchstate_status(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(8);
  pkg_headerext_status_set_messagegroupid(20);
  pkg_headerext_status_set_messageid(1);
  __HEADEROFFSETBITS = 83;
  __PACKETSIZEBYTES = 16;
  __MESSAGETYPE = 8;
}

// Function to initialize header for the MessageType "Ack".
static inline void pkg_header_init_powerswitch_switchstate_ack(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(9);
  __HEADEROFFSETBITS = 109;
  __PACKETSIZEBYTES = 16;
  __MESSAGETYPE = 9;
}

// Function to initialize header for the MessageType "AckStatus".
static inline void pkg_header_init_powerswitch_switchstate_ackstatus(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(10);
  pkg_headerext_ackstatus_set_messagegroupid(20);
  pkg_headerext_ackstatus_set_messageid(1);
  __HEADEROFFSETBITS = 120;
  __PACKETSIZEBYTES = 32;
  __MESSAGETYPE = 10;
}

// On (BoolValue)
// Description: Tells if the switch is on (active).

// Set On (BoolValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 0) / 8, ((uint16_t)__HEADEROFFSETBITS + 0) % 8, length bits 1
static inline void msg_powerswitch_switchstate_set_on(bool val)
{
  array_write_UIntValue(((uint16_t)__HEADEROFFSETBITS + 0) / 8, ((uint16_t)__HEADEROFFSETBITS + 0) % 8, 1, val ? 1 : 0, bufx);
}

// Get On (BoolValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 0) / 8, ((uint16_t)__HEADEROFFSETBITS + 0) % 8, length bits 1
static inline bool msg_powerswitch_switchstate_get_on(void)
{
  return array_read_UIntValue8(((uint16_t)__HEADEROFFSETBITS + 0) / 8, ((uint16_t)__HEADEROFFSETBITS + 0) % 8, 1, 0, 1, bufx) == 1;
}

// TimeoutSec (UIntValue)
// Description: The time after which the switch is automatically toggled again. Use 0 to disable this.

// Set TimeoutSec (UIntValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 1) % 8, length bits 16, min val 0, max val 65535
static inline void msg_powerswitch_switchstate_set_timeoutsec(uint32_t val)
{
  array_write_UIntValue(((uint16_t)__HEADEROFFSETBITS + 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 1) % 8, 16, val, bufx);
}

// Get TimeoutSec (UIntValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 1) % 8, length bits 16, min val 0, max val 65535
static inline uint32_t msg_powerswitch_switchstate_get_timeoutsec(void)
{
  return array_read_UIntValue32(((uint16_t)__HEADEROFFSETBITS + 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 1) % 8, 16, 0, 65535, bufx);
}


// Message "powerswitch_switchstateext"
// ------------------------------------
// MessageGroupID: 20
// MessageID: 2
// Possible MessageTypes: Get, Set, SetGet, Status, Ack, AckStatus
// Validity: test
// Length w/o Header + HeaderExtension: 144 bits
// Data fields: On, TimeoutSec
// Description: This is the state of up to 8 relais and its timeout values.

// Function to initialize header for the MessageType "Get".
static inline void pkg_header_init_powerswitch_switchstateext_get(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(0);
  pkg_headerext_get_set_messagegroupid(20);
  pkg_headerext_get_set_messageid(2);
  __HEADEROFFSETBITS = 95;
  __PACKETSIZEBYTES = 16;
  __MESSAGETYPE = 0;
}

// Function to initialize header for the MessageType "Set".
static inline void pkg_header_init_powerswitch_switchstateext_set(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(1);
  pkg_headerext_set_set_messagegroupid(20);
  pkg_headerext_set_set_messageid(2);
  __HEADEROFFSETBITS = 95;
  __PACKETSIZEBYTES = 32;
  __MESSAGETYPE = 1;
}

// Function to initialize header for the MessageType "SetGet".
static inline void pkg_header_init_powerswitch_switchstateext_setget(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(2);
  pkg_headerext_setget_set_messagegroupid(20);
  pkg_headerext_setget_set_messageid(2);
  __HEADEROFFSETBITS = 95;
  __PACKETSIZEBYTES = 32;
  __MESSAGETYPE = 2;
}

// Function to initialize header for the MessageType "Status".
static inline void pkg_header_init_powerswitch_switchstateext_status(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(8);
  pkg_headerext_status_set_messagegroupid(20);
  pkg_headerext_status_set_messageid(2);
  __HEADEROFFSETBITS = 83;
  __PACKETSIZEBYTES = 32;
  __MESSAGETYPE = 8;
}

// Function to initialize header for the MessageType "Ack".
static inline void pkg_header_init_powerswitch_switchstateext_ack(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(9);
  __HEADEROFFSETBITS = 109;
  __PACKETSIZEBYTES = 16;
  __MESSAGETYPE = 9;
}

// Function to initialize header for the MessageType "AckStatus".
static inline void pkg_header_init_powerswitch_switchstateext_ackstatus(void)
{
  memset(&bufx[0], 0, sizeof(bufx));
  pkg_header_set_messagetype(10);
  pkg_headerext_ackstatus_set_messagegroupid(20);
  pkg_headerext_ackstatus_set_messageid(2);
  __HEADEROFFSETBITS = 120;
  __PACKETSIZEBYTES = 48;
  __MESSAGETYPE = 10;
}

// On (BoolValue[8])

// Set On (BoolValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) % 8, length bits 1
static inline void msg_powerswitch_switchstateext_set_on(uint8_t index, bool val)
{
  array_write_UIntValue(((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) % 8, 1, val ? 1 : 0, bufx);
}

// Get On (BoolValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) % 8, length bits 1
static inline bool msg_powerswitch_switchstateext_get_on(uint8_t index)
{
  return array_read_UIntValue8(((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) / 8, ((uint16_t)__HEADEROFFSETBITS + 0 + (uint16_t)index * 1) % 8, 1, 0, 1, bufx) == 1;
}

// TimeoutSec (UIntValue[8])
// Description: The time after which the switch is automatically toggled again. Use 0 to disable this.

// Set TimeoutSec (UIntValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) / 8, ((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) % 8, length bits 16, min val 0, max val 65535
static inline void msg_powerswitch_switchstateext_set_timeoutsec(uint8_t index, uint32_t val)
{
  array_write_UIntValue(((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) / 8, ((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) % 8, 16, val, bufx);
}

// Get TimeoutSec (UIntValue)
// Offset: ((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) / 8, ((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) % 8, length bits 16, min val 0, max val 65535
static inline uint32_t msg_powerswitch_switchstateext_get_timeoutsec(uint8_t index)
{
  return array_read_UIntValue32(((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) / 8, ((uint16_t)__HEADEROFFSETBITS + 8 + (uint16_t)index * 16) % 8, 16, 0, 65535, bufx);
}

#endif /* _MSGGRP_POWERSWITCH_H */
