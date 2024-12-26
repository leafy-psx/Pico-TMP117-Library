/**
 * Copyright (c) 2024 breaker
 * Based on code from:
 * - Raspberry Pi Pico SDK (c) 2020 Raspberry Pi (Trading) Ltd. (BSD-3-Clause License)
 * - SparkFun TMP117 Arduino Library (c) 2016 SparkFun Electronics (originally MIT, relicensed BSD-3-Clause)
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Original SparkFun TMP117 Arduino Library header:

/******************************************************************************
SparkFun_TMP117_Registers.h
TMP117 Library - TMP117 Register Map
Madison Chodikov @ SparkFun Electronics
Original Creation Date: April 19, 2019
https://github.com/sparkfunX/Qwiic_TMP117

This file defines all registers internal to the TMP117 sensor.

Development environment specifics:
    IDE: Arduino 1.8.9
    Hardware Platform: Arduino Uno
    TMP117 Breakout Version: 1.0.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

// This version is for use as an INTERFACE library to the Raspberry Pi Pico SDK

#ifndef TMP117_Registers_H
#define TMP117_Registers_H

// From TMP117 Register Map, datasheet table 7-3
enum TMP117_register
{
  TMP117_TEMP_RESULT = 0X00,
  TMP117_CONFIGURATION = 0x01,
  TMP117_T_HIGH_LIMIT = 0X02,
  TMP117_T_LOW_LIMIT = 0X03,
  TMP117_EEPROM_UL = 0X04,
  TMP117_EEPROM1 = 0X05,
  TMP117_EEPROM2 = 0X06,
  TMP117_TEMP_OFFSET = 0X07,
  TMP117_EEPROM3 = 0X08,
  TMP117_DEVICE_ID = 0X0F
};

#endif
