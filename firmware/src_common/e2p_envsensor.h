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

#ifndef _E2P_ENVSENSOR_H
#define _E2P_ENVSENSOR_H

// E2P Block "EnvSensor"
// =====================
// Start offset (bit): 512
// Overall block length: 7680 bits

// TemperatureSensorType (EnumValue)
// Description: You can choose one of the supported temperature / humidity sensors. If set to 0, no sensor is used, but the device sends out packets for testing purposes.

typedef enum {
  TEMPERATURESENSORTYPE_NOSENSOR = 0,
  TEMPERATURESENSORTYPE_SHT15 = 1,
  TEMPERATURESENSORTYPE_DS7505 = 2
} TemperatureSensorTypeEnum;

// Set TemperatureSensorType (EnumValue)
// Byte offset: 64, bit offset: 0, length bits 8
static inline void e2p_envsensor_set_temperaturesensortype(TemperatureSensorTypeEnum val)
{
  eeprom_write_UIntValue(64, 0, 8, val);
}

// Get TemperatureSensorType (EnumValue)
// Byte offset: 64, bit offset: 0, length bits 8
static inline TemperatureSensorTypeEnum e2p_envsensor_get_temperaturesensortype(void)
{
  return eeprom_read_UIntValue8(64, 0, 8, 0, 255);
}

// BrightnessSensorType (EnumValue)
// Description: You can choose one of the supported light sensors. If set to 0, no sensor is used, but the device sends out packets for testing purposes.

typedef enum {
  BRIGHTNESSSENSORTYPE_NOSENSOR = 0,
  BRIGHTNESSSENSORTYPE_PHOTOCELL = 1
} BrightnessSensorTypeEnum;

// Set BrightnessSensorType (EnumValue)
// Byte offset: 65, bit offset: 0, length bits 8
static inline void e2p_envsensor_set_brightnesssensortype(BrightnessSensorTypeEnum val)
{
  eeprom_write_UIntValue(65, 0, 8, val);
}

// Get BrightnessSensorType (EnumValue)
// Byte offset: 65, bit offset: 0, length bits 8
static inline BrightnessSensorTypeEnum e2p_envsensor_get_brightnesssensortype(void)
{
  return eeprom_read_UIntValue8(65, 0, 8, 0, 255);
}

typedef enum {
  BAROMETRICSENSORTYPE_NOSENSOR = 0,
  BAROMETRICSENSORTYPE_BMP085 = 1
} BarometricSensorTypeEnum;

// Set BarometricSensorType (EnumValue)
// Byte offset: 66, bit offset: 0, length bits 8
static inline void e2p_envsensor_set_barometricsensortype(BarometricSensorTypeEnum val)
{
  eeprom_write_UIntValue(66, 0, 8, val);
}

// Get TemperatureSensorType (EnumValue)
// Byte offset: 64, bit offset: 0, length bits 8
static inline TemperatureSensorTypeEnum e2p_envsensor_get_barometricsensortype(void)
{
  return eeprom_read_UIntValue8(66, 0, 8, 0, 255);
}


// Reserved area with 7664 bits


#endif /* _E2P_ENVSENSOR_H */
