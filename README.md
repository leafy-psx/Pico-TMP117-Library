TMP117 library for Raspberry Pi Pico C SDK
================================================================================
This version is for use as an INTERFACE library to the Raspberry Pi Pico SDK

Based on the SparkFun TMP117 Arduino Library and ported to the Raspberry Pi Pico C SDK by Madison Chodikov.
The basics of the original Library remains but much of it has changed in this version.
This port is written in C, by breaker (name on the Raspberry Pi Forum).
This was written to use my sensor breakout with the SDK and learn it a bit along the way.

Tested with the Raspberry Pi Pico H, Pimoroni Pico Plus 2 (RP2350), and SparkFun SEN-15805.
Texas Instruments TMP117 High-Accuracy, Low-Power, Digital Temperature Sensor With SMBus™- and I2C-Compatible Interface
data sheet: https://www.ti.com/lit/gpn/TMP117
SparkFun SEN-15805: https://www.sparkfun.com/products/15805
Refer to the Raspberry Pico documentation to set up the Pico C SDK build environment.
https://www.raspberrypi.com/documentation/microcontrollers/

Notes about the code:
================================================================================
There are 13 examples, some are in the same directory.
All examples built and ran OK using Pico SDK 2.1.0 for both RP2040 and RP2350 boards.
Default Pico board file I2C and UART pins were used. Configure alert interrupt pin in examples.

In the first temperature reading example temp_result.c
Q notation is introduced in order to show how the temperature can be displayed without using floating point math. 

See TI Application Note: How to Read and Interpret Digital Temperature Sensor Output Data
"Modern sensors, such as the TMP117, offer a full 16 bits of resolution in a Q7 format."

The 16-bit Word of the temp_result register consists of;
bit 15: sign bit, [14:7] Integer bits, "decimal point", [6:0] Fractional bits
================================================================================

Defined types and enumerations were added for many function parameters, review tmp117.h for details.

To read from a register the pointer register must be written to first. Therefore the pointer register does not need to be written to if this was the last register written to. The updated read_register function keeps track of the last pointer written to, in order to avoid writing to the pointer register unnecessarily.

Writes to the registers are volatile unless the EEPROM is programmed.

Many new functions were added to the library including the raw temperature register read for the Q notation display and a function and example for the SMB alert response, and functions and examples for EEPROM programming.

A structure was added to handle the reading of the configuration register and populating the high alert, low alert, and data ready flags all at once. This was necessary to test one-shot mode with alert flag status because if the configuration register is read then the data ready flag is cleared. Both one shot mode and continuous conversion mode use the data ready flag despite a comment in the original library example.

It is important to note that reading the configuration register clears the high alert and low alert flags.
Also, reading the configuration register or the temperature result register clears the data ready flag.

Therefore when the temperature register is populated with a specific reading the data ready flag is then set and any applicable alert flag is set. We need to be careful not to read the temperature result register before reading the configuration register in one-shot mode or else the data ready flag will be cleared before it can be used to detect a ready temp_result register.

The EEPROM factory default is 8 temperature conversions averaged per second.
Averaging can be used in both the continuous conversion mode and one-shot mode
See section 7.3.2 Averaging in the datasheet for details

NOTE: The EEPROM have a finite amount of writes in the range of as low as 1000, but typically 50,000.
Also, you may want to record the values of the general purpose EEPROM before programming, see NIST traceability in the datasheet.

Possible i2C addresses for TMP117 (0x48, 0x49, 0x4A, 0x4B) (set by wiring jumpers on the Sparkfun board).
Library assumes the address is 0x48 unless set_address function is used.

NOTE: The device requires 1.5 ms to power up before conversions can begin.

/src - library source and header files
/examples - various examples

A big thank you Madison Chodikov @ SparkFun Electronics for writing the SparkFun TMP117 Qwiic Arduino library and board design.

See LICENSE.txt for license details.
