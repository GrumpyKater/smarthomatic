/*
    Copyright (c) 2015 Holger Gross

    Code based on
    	- smarthomatic Copyright (c) 2013 Uwe Freese
    	- OpenHr20 Copyright (c) 2008 Dario Carluccio (hr20-at-carluccio-dot-de)
								 2008 Jiri Dobry (jdobry-at-centrum-dot-cz)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

#include "rfm12.h"
#include "../src_common/uart.h"

#include "../src_common/msggrp_generic.h"
#include "../src_common/msggrp_gpio.h"
#include "../src_common/msggrp_thermostat.h"

#include "../src_common/e2p_hardware.h"
#include "../src_common/e2p_generic.h"
#include "../src_common/e2p_thermostat.h"

#include "../src_common/aes256.h"
#include "../src_common/util.h"
#include "version.h"

uint16_t device_id;
uint32_t station_packetcounter;

#define	MOTOR_MAX_IMPULSES 1000
#define	MOTOR_HYSTERESIS     50

#define MOTOR_eye_enable() PORTE |= _BV(PE3);   // activate photo eye
#define MOTOR_eye_disable() PORTE &= ~_BV(PE3); // deactivate photo eye

volatile uint16_t MOTOR_PosAct;      //!< actual position
volatile uint16_t MOTOR_PosMax;      //!< position if complete open (100%), 0 if not calibrated
volatile uint16_t MOTOR_PosStop;     //!< stop at this position
volatile uint16_t MOTOR_PosLast;     //!< last position for check if blocked

volatile bool MOTOR_Mounted = true; // TODO

typedef enum {
	stop, open, close
} motor_dir_t;

volatile motor_dir_t MOTOR_Dir; 	 //!< actual direction

static inline void MOTOR_H_BRIDGE_open(void) {
	MOTOR_Dir = open;
	PORTE |= (1<<PE3);    // activate photo eye
	PCMSK0 = (1<<PCINT4); // activate interrupt
	PORTG  =  (1<<PG4);   // PG3 LOW, PG4 HIGH
	PORTB |=  (1<<PB7);   // PB7 HIGH
	PORTB &= ~(1<<PB4);   // PB7 LOW
}

static inline void MOTOR_H_BRIDGE_close(void) {
	MOTOR_Dir = close;
	PORTE |= (1<<PE3);    // activate photo eye
	PCMSK0 = (1<<PCINT4); // activate interrupt
	PORTG  =  (1<<PG3);   // PG3 HIGH, PG4 LOW
	PORTB &= ~(1<<PB7);   // PB7 LOW
	PORTB |=  (1<<PB4);   // PB4 HIGH
}

static inline void MOTOR_H_BRIDGE_stop(void) {
	UART_PUTS("========= STOP =========\r\n");
	UART_PUTF("MOTOR_PosStop %u\r\n", MOTOR_PosStop);
	UART_PUTF("MOTOR_PosMax %u\r\n", MOTOR_PosMax);
	MOTOR_Dir = stop;
	PCMSK0 &= ~(1<<PCINT4);   // deactivate interrupt
	PORTE &= ~(1<<PE3);       // deactivate photo eye
	PORTG  =  0;          // PG3 LOW, PG4 LOW
	PORTB &= ~(1<<PB7);   // PB7 LOW
	PORTB &= ~(1<<PB4);   // PB7 LOW
}

ISR (PCINT0_vect) {
	// count only on HIGH impulses
	if (((PINE & (1<<PE4)) != 0)) {
		//UART_PUTS("((PINE & (1<<PE4)) != 0)\r\n");
		if (MOTOR_Dir == open) {
			MOTOR_PosAct++;
			if (!(MOTOR_PosAct < MOTOR_PosStop)) {
				UART_PUTS("========= !(MOTOR_PosAct < MOTOR_PosStop) =========\r\n");
				UART_PUTF("MOTOR_PosAct %u\r\n", MOTOR_PosAct);
				UART_PUTF("MOTOR_PosStop %u\r\n", MOTOR_PosStop);
				MOTOR_H_BRIDGE_stop();
			}
		} else {
			MOTOR_PosAct--;
			if (!(MOTOR_PosAct > MOTOR_PosStop)) {
				UART_PUTS("========= !(MOTOR_PosAct > MOTOR_PosStop) =========\r\n");
				UART_PUTF("MOTOR_PosAct %u\r\n", MOTOR_PosAct);
				UART_PUTF("MOTOR_PosStop %u\r\n", MOTOR_PosStop);
				MOTOR_H_BRIDGE_stop();
			}
		}
	}
}

void MOTOR_Init(void)
{
	MOTOR_PosAct=0;
	MOTOR_PosMax=760; // TODO
	MOTOR_PosStop=0;
	MOTOR_H_BRIDGE_stop();
}

bool MOTOR_IsCalibrated(void)
{
	return (MOTOR_PosMax != 0);
}

bool MOTOR_On(void)
{
	return (MOTOR_Dir != stop);
}

void MOTOR_CheckBlocked(void){
	UART_PUTF("MOTOR_PosAct %u\r\n", MOTOR_PosAct);
	UART_PUTF("MOTOR_PosLast %u\r\n", MOTOR_PosLast);
	if (MOTOR_PosAct == MOTOR_PosLast){
		UART_PUTS("========= BLOCKED =========\r\n");
		UART_PUTF("MOTOR_PosAct %u\r\n", MOTOR_PosAct);
		UART_PUTF("MOTOR_PosLast %u\r\n", MOTOR_PosLast);
		MOTOR_H_BRIDGE_stop();
	} else {
		UART_PUTS("not blocked\r\n");
		MOTOR_PosLast = MOTOR_PosAct;
	}
}

bool MOTOR_Goto(uint8_t percent)
{
	UART_PUTS("========= GOTO =========\r\n");
	UART_PUTF("percent %u\r\n", percent);
	if (MOTOR_IsCalibrated()){
		if (percent == 100) {
			MOTOR_PosStop = MOTOR_PosMax + MOTOR_HYSTERESIS;
		} else if (percent == 0) {
			MOTOR_PosStop = 0;
			MOTOR_PosAct += MOTOR_HYSTERESIS;
		} else {
			MOTOR_PosStop = (uint16_t) ((percent/10) * (MOTOR_PosMax / 10));
			UART_PUTF("MOTOR_PosStop %u\r\n", MOTOR_PosStop);
			UART_PUTF("MOTOR_PosMax %u\r\n", MOTOR_PosMax);
		}
		if (MOTOR_PosAct > MOTOR_PosStop){
			MOTOR_H_BRIDGE_close();
		} else if (MOTOR_PosAct < MOTOR_PosStop){
			MOTOR_H_BRIDGE_open();
		}
		UART_PUTS("true\r\n");
		return true;
	} else {
		UART_PUTS("false\r\n");
		return false;
	}
}

bool MOTOR_Calibrate(uint8_t percent)
{
	uint16_t postmp;
	if (percent > 50) {
		// - close till no movement or more than MOTOR_MAX_IMPULSES
		MOTOR_PosAct = MOTOR_MAX_IMPULSES;
		MOTOR_PosStop = 0;
		MOTOR_H_BRIDGE_close();
		do {
			postmp = MOTOR_PosAct;
			_delay_ms(100);
		} while ((MOTOR_Dir != stop) && !((postmp == MOTOR_PosAct)||(postmp == MOTOR_PosAct-1)||(postmp == MOTOR_PosAct-2)||(postmp == MOTOR_PosAct-3)) && MOTOR_Mounted);
		// stopped by ISR?, turning too long, not mounted -> error
		if ((MOTOR_Dir == stop) || (MOTOR_Mounted == false)) {
			return false;
		}
		// motor is on, but not turning any more -> endposition reached -> stop the motor
		MOTOR_H_BRIDGE_stop();
		// now open till no movement or more than MOTOR_MAX_IMPULSES
		MOTOR_PosAct = 0;
		MOTOR_PosStop = MOTOR_MAX_IMPULSES;
		MOTOR_H_BRIDGE_open();
		do {
			postmp = MOTOR_PosAct;
			_delay_ms(100);
		} while ((MOTOR_Dir != stop) && !((postmp == MOTOR_PosAct)||(postmp == MOTOR_PosAct+1)||(postmp == MOTOR_PosAct+2)||(postmp == MOTOR_PosAct+3)) && MOTOR_Mounted);
		// stopped by ISR?, turning too long, not mounted -> error
		if ((MOTOR_Dir == stop) || (MOTOR_Mounted == false)) {
			return false;
		}
		// motor is on, but not turning any more -> endposition reached -> stop the motor
		MOTOR_H_BRIDGE_stop();
		MOTOR_PosMax = MOTOR_PosAct;
	} else {
		// - open till no movement or more than MOTOR_MAX_IMPULSES
		MOTOR_PosAct = 0;
		MOTOR_PosStop = MOTOR_MAX_IMPULSES;
		MOTOR_H_BRIDGE_open();
		do {
			postmp = MOTOR_PosAct;
			//UART_PUTF("do1 postmp %u\r\n", postmp);
			_delay_ms(100);
			//UART_PUTF("- do1 %u\r\n", MOTOR_PosAct);
		} while ((MOTOR_Dir != stop) && !((postmp == MOTOR_PosAct)||(postmp == MOTOR_PosAct-1)||(postmp == MOTOR_PosAct-2)||(postmp == MOTOR_PosAct-3)) && MOTOR_Mounted);
		UART_PUTS("after do1\r\n");
		UART_PUTF("MOTOR_PosAct %u\r\n", MOTOR_PosAct);
		UART_PUTF("MOTOR_PosStop %u\r\n", MOTOR_PosStop);
		// stopped by ISR?, turning too long, not mounted -> error
		if ((MOTOR_Dir == stop) || (MOTOR_Mounted==false)) {
			//UART_PUTS("after do1 if\r\n");
			return false;
		}
		// motor is on, but not turning any more -> endposition reached -> stop the motor
		MOTOR_H_BRIDGE_stop();
		// now close till no movement or more than MOTOR_MAX_IMPULSES
		MOTOR_PosAct = MOTOR_MAX_IMPULSES;
		MOTOR_PosStop = 0;
		MOTOR_H_BRIDGE_close();
		do {
			postmp = MOTOR_PosAct;
			//UART_PUTF("do2 postmp %u\r\n", postmp);
			_delay_ms(100);
			//UART_PUTF("- do2 %u\r\n", MOTOR_PosAct);
		} while ((MOTOR_Dir != stop) && !((postmp == MOTOR_PosAct)||(postmp == MOTOR_PosAct+1)||(postmp == MOTOR_PosAct+2)||(postmp == MOTOR_PosAct+3)) && MOTOR_Mounted);
		UART_PUTS("after do2\r\n");
		UART_PUTF("MOTOR_PosAct %u\r\n", MOTOR_PosAct);
		UART_PUTF("MOTOR_PosStop %u\r\n", MOTOR_PosStop);
		// stopped by ISR?, turning too long, not mounted -> error
		if ((MOTOR_Dir == stop) || (MOTOR_Mounted==false)){
			//UART_PUTS("after do2 if\r\n");
			return false;
		}
		//UART_PUTS("true\r\n");
		MOTOR_H_BRIDGE_stop();
		MOTOR_PosMax = MOTOR_MAX_IMPULSES - MOTOR_PosAct;
		MOTOR_PosAct = 0;
	}
	UART_PUTF("MOTOR_PosAct %u\r\n", MOTOR_PosAct);
	UART_PUTF("MOTOR_PosStop %u\r\n", MOTOR_PosStop);
	UART_PUTF("MOTOR_PosMax %u\r\n", MOTOR_PosMax);

	MOTOR_Goto(0);
	return true;
}

void send_deviceinfo_status(void)
{
	inc_packetcounter();

	UART_PUTF("Send DeviceInfo: DeviceType %u,", DEVICETYPE_THERMOSTAT);
	UART_PUTF4(" v%u.%u.%u (%08lx)\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_HASH);
	
	// Set packet content
	pkg_header_init_generic_deviceinfo_status();
	pkg_header_set_senderid(device_id);
	pkg_header_set_packetcounter(packetcounter);
	msg_generic_deviceinfo_set_devicetype(DEVICETYPE_THERMOSTAT);
	msg_generic_deviceinfo_set_versionmajor(VERSION_MAJOR);
	msg_generic_deviceinfo_set_versionminor(VERSION_MINOR);
	msg_generic_deviceinfo_set_versionpatch(VERSION_PATCH);
	msg_generic_deviceinfo_set_versionhash(VERSION_HASH);

	rfm12_send_bufx();
}

void send_ack(uint32_t acksenderid, uint32_t ackpacketcounter, bool error)
{
	// any message can be used as ack, because they are the same anyway
	if (error)
	{
		UART_PUTS("Send error Ack\r\n");
		pkg_header_init_gpio_digitalporttimeout_ack(); // TODO
	}

	inc_packetcounter();
	
	// set common fields
	pkg_header_set_senderid(device_id);
	pkg_header_set_packetcounter(packetcounter);
	
	pkg_headerext_common_set_acksenderid(acksenderid);
	pkg_headerext_common_set_ackpacketcounter(ackpacketcounter);
	pkg_headerext_common_set_error(error);
	
	rfm12_send_bufx();
}

// Process a request to this device.
// React accordingly on the MessageType, MessageGroup and MessageID
// and send an Ack in any case. It may be an error ack if request is not supported.
void process_request(MessageTypeEnum messagetype, uint32_t messagegroupid, uint32_t messageid)
{
	// remember some values before the packet buffer is destroyed
	uint32_t acksenderid = pkg_header_get_senderid();
	uint32_t ackpacketcounter = pkg_header_get_packetcounter();
	
	UART_PUTF("MessageGroupID:%u;", messagegroupid);
	
	if (messagegroupid != MESSAGEGROUP_THERMOSTAT)
	{
		UART_PUTS("\r\nERR: Unsupported MessageGroupID.\r\n");
		send_ack(acksenderid, ackpacketcounter, true);
		return;
	}
	
	UART_PUTF("MessageID:%u;", messageid);

	switch (messageid)
	{
		case MESSAGEID_THERMOSTAT_STATUS:
			UART_PUTS("MESSAGEID_THERMOSTAT_STATUS\r\n");

			uint32_t position = msg_thermostat_status_get_valveposition();
			UART_PUTF("MESSAGEID_THERMOSTAT_STATUS: %u\r\n", position);

			switch (position) {
				case 0:
					MOTOR_H_BRIDGE_stop();
					break;
				case 1:
					MOTOR_H_BRIDGE_close();
					break;
				case 2:
					MOTOR_H_BRIDGE_open();
					break;
				case 3:
					if (MOTOR_Calibrate(3)) {
						UART_PUTS("MOTOR_Calibrate true\r\n");
					} else {
						UART_PUTS("MOTOR_Calibrate false\r\n");
					}
					break;
				default:
					MOTOR_Goto(position);
			}
			UART_PUTS("done\r\n");
			break;
		default:
			UART_PUTS("\r\nERR: Unsupported MessageID.");
			send_ack(acksenderid, ackpacketcounter, true);
			return;
	}
	
	UART_PUTS("\r\n");

	// In all cases, use the digitalporttimer message as answer.
	// It contains the data for *all* pins and *all* timer values.

	// "Set" -> send "Ack"
	if (messagetype == MESSAGETYPE_SET)
	{
		pkg_header_init_thermostat_status_ack();

		UART_PUTS("Sending Ack\r\n");
	}
	// "Get" or "SetGet" -> send "AckStatus"
	else
	{
		pkg_header_init_thermostat_status_ackstatus();
		
		UART_PUTS("Sending AckStatus\r\n");
	}

	send_ack(acksenderid, ackpacketcounter, false);
}

// Check if incoming message is a legitimate request for this device.
// If not, ignore it.
void process_packet(uint8_t len)
{
	pkg_header_adjust_offset();

	// check SenderID
	uint32_t senderID = pkg_header_get_senderid();
	UART_PUTF("Packet Data: SenderID:%u;", senderID);
	
	if (senderID != 0)
	{
		UART_PUTS("\r\nERR: Illegal SenderID.\r\n");
		return;
	}

	// check PacketCounter
	// TODO: Reject if packet counter lower than remembered!!
	uint32_t packcnt = pkg_header_get_packetcounter();
	UART_PUTF("PacketCounter:%lu;", packcnt);

	if (0) // packcnt <= station_packetcounter ??
	{
		UART_PUTF("\r\nERR: Received PacketCounter < %lu.\r\n", station_packetcounter);
		return;
	}
	
	// write received counter
	station_packetcounter = packcnt;
	
	e2p_thermostat_set_basestationpacketcounter(station_packetcounter);
	
	// check MessageType
	MessageTypeEnum messagetype = pkg_header_get_messagetype();
	UART_PUTF("MessageType:%u;", messagetype);
	
	if ((messagetype != MESSAGETYPE_GET) && (messagetype != MESSAGETYPE_SETGET))
	{
		UART_PUTS("\r\nERR: Unsupported MessageType.\r\n");
		return;
	}
	
	// check device id
	uint16_t rcv_id = pkg_headerext_common_get_receiverid();

	UART_PUTF("ReceiverID:%u;", rcv_id);
	
	if (rcv_id != device_id)
	{
		UART_PUTS("\r\nWRN: DeviceID does not match.\r\n");
		return;
	}
	
	// check MessageGroup + MessageID
	uint32_t messagegroupid = pkg_headerext_common_get_messagegroupid();
	uint32_t messageid = pkg_headerext_common_get_messageid();
	
	process_request(messagetype, messagegroupid, messageid);
}

int main(void)
{
	uint8_t loop = 0;

	//! set Clock to 4 Mhz // TODO do we need 4Mhz?
	CLKPR = (1<<CLKPCE);            // prescaler change enable
	CLKPR = (1<<CLKPS0);            // prescaler = 2 (internal RC runs @ 8MHz)

	//! Disable Analog Comparator (power save)
	//ACSR = (1<<ACD);

	//! digital I/O port direction
	DDRG = (1<<PG3)|(1<<PG4); // PG3, PG4 Motor out
	PORTB = (0<<PB6);
	DDRB = (1<<PB4)|(1<<PB7)|(1<<PB6); // PB4, PB7 Motor out
	DDRE = (1<<PE3); // PE3  activate lighteye

	// activate PCINT0 (PCINT7:0 pins)
	EIMSK = (1<<PCIE0);

	MOTOR_Init();

	// delay 1s to avoid further communication with uart or RFM12 when my programmer resets the MC after 500ms...
	_delay_ms(1000);

	util_init();
	
	//check_eeprom_compatibility(DEVICETYPE_THERMOSTAT);

	// read packetcounter, increase by cycle and write back
	packetcounter = e2p_generic_get_packetcounter() + PACKET_COUNTER_WRITE_CYCLE;
	e2p_generic_set_packetcounter(packetcounter);

	// read last received station packetcounter
	//station_packetcounter = e2p_hr20_get_basestationpacketcounter();
	station_packetcounter = 0;
	
	// read device id
	device_id = 99;
	//device_id = e2p_generic_get_deviceid();

	osccal_init();

	uart_init();

	UART_PUTS ("\r\n");
	osccal_info();
	UART_PUTF ("DeviceID: %u\r\n", device_id);
	UART_PUTF ("PacketCounter: %lu\r\n", packetcounter);
	UART_PUTF ("Last received base station PacketCounter: %u\r\n\r\n", station_packetcounter);
	
	// init AES key
	e2p_generic_get_aeskey(aes_key);

	rfm12_init();

	sei();

	send_deviceinfo_status();

	while (1)
	{
		if (loop == 50 && MOTOR_On()) {
			MOTOR_CheckBlocked();
			loop = 0;
		}

		if (rfm12_rx_status() == STATUS_COMPLETE)
		{
			UART_PUTS("STATUS_COMPLETE\r\n");
			uint8_t len = rfm12_rx_len();
			
			if ((len == 0) || (len % 16 != 0))
			{
				UART_PUTF("Received garbage (%u bytes not multiple of 16): ", len);
				print_bytearray(bufx, len);
			}
			else // try to decrypt with all keys stored in EEPROM
			{
				memcpy(bufx, rfm12_rx_buffer(), len);
				
				UART_PUTS("Before decryption: ");
				print_bytearray(bufx, len);
					
				aes256_decrypt_cbc(bufx, len);

				UART_PUTS("Decrypted bytes: ");
				print_bytearray(bufx, len);

				if (!pkg_header_check_crc32(len))
				{
					UART_PUTS("Received garbage (CRC wrong after decryption).\r\n");
				}
				else
				{
					UART_PUTS("process_packet\r\n");
					process_packet(len);
				}
			}

			rfm12_rx_clear();
		}
		loop++;

		rfm12_tick();	
	}
}
