// temp_result.c
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
  Example1_BasicReadings.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  May 29 2019
  ~

  This sketch configures the TMP117 temperature sensor and prints the
  temperature in degrees celsius and fahrenheit with a 500ms delay for
  easier readings. 

  Resources:
  Wire.h (included with Arduino IDE)
  SparkFunTMP117.h (included in the src folder) http://librarymanager/All#SparkFun_TMP117

  Development environment specifics:
  Arduino 1.8.9+
  Hardware Version 1.0.0

  This code is beerware; if you see me (or any other SparkFun employee) at
  the local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given.
******************************************************************************/

/*
  NOTE: For the most accurate readings:
  - Avoid heavy bypass traffic on the I2C bus
  - Use the highest available communication speeds
  - Use the minimal supply voltage acceptable for the system
  - Place device horizontally and out of any airflow when storing
  For more information on reaching the most accurate readings from the sensor,
  reference the "Precise Temperature Measurements with TMP116" datasheet that is
  linked on Page 35 of the TMP117's datasheet
*/

/*** Notes for this new version using the Raspberry Pi Pico/RP2040: ***/

/* temp_result.c
*
* This program will print the temperature in Celsius by casting the raw temperature
* result register to an integer, then scale the Q7 formatted data to two decimal places
* 
* See TI Application Note: How to Read and Interpret Digital Temperature Sensor Output Data
* "Modern sensors, such as the TMP117, offer a full 16 bits of resolution in a Q7 format."
*
* The 16-bit Word of the temp_result register consists of;
* bit 15: sign bit, [14:7] Integer bits, "decimal point", [6:0] Fractional bits
*
* Please see Table 7-7. (Conversion Cycle Time in CC Mode) note: CC = Continuous Conversion
* The factory default Conversion Cycle time is 1 second using 8 averaged conversions,
* therefore this example sets the wait for conversion to 1000 ms.
*
* NOTE: Possible i2C addresses for TMP117 (0x48, 0x49, 0x4A, 0x4B)
* Library assumes the address is 0x48 unless set_address function is used
*
* - Reading the configuration register clears the High Alert and Low Alert flags
* - Reading the configuration or temperature result registers clears the Data Ready flag.
*/

#include "tmp117.h"
#include "tmp117_registers.h"
#include "pico/stdlib.h"
#include "pico/printf.h"
#include "hardware/i2c.h"
#include <stdbool.h>
#include <stdint.h>

#define SERIAL_INIT_DELAY_MS 1000 // adjust as needed to mitigate garbage characters after serial interface is started
#define TMP117_I2C_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN // set to a different SDA pin as needed
#define TMP117_I2C_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN // set to a different SCL pin as needed
#define TMP117_OFFSET_VALUE -25.0f  // temperature offset in degrees C set by user (try negative values for testing)
#define TMP117_CONVERSION_DELAY_MS 1000 // Adjust the delay based on conversion cycle time and preference

void check_i2c(void);
void check_status(void);

// variable for checking I2C frequency
uint frequency = 0;

int main(void) {
    // initialize chosen interface
    stdio_init_all();
    // a little delay to ensure serial line stability
    sleep_ms(SERIAL_INIT_DELAY_MS);

    // uncomment below to set I2C address other than 0x48 (e.g., 0x49)
    //tmp117_set_address(0x49);

    // Selects I2C instance (i2c0 is set as default in the tmp117.c)
    //tmp117_set_instance(i2c1); // change to i2c1 as needed

    // initialize I2C (default i2c0) and initialize variable with I2C frequency
    frequency = i2c_init(i2c_instance, 400 * 1000); // TMP117 400 kHz max.

    // configure the GPIO pins for I2C
    gpio_set_function(TMP117_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(TMP117_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // call a function to check and report if i2c is running
    check_i2c();
 
    // check if TMP117 is on the I2C bus at the address specified
    check_status();

    // TMP117 software reset; loads EEPROM Power On Reset values
    soft_reset();

    while (1) {

        do {
            sleep_ms(TMP117_CONVERSION_DELAY_MS);
        } while (!data_ready()); // check if the data ready flag is high

        /* 1) typecast temp_result register to integer, converting from two's complement
         * 2) Multiply by 100 to scale the temperature (i.e. 2 decimal places)
         * 3) Shift right by 7 to account for the TMP117's 1/128 resolution (Q7 format), converting to degrees Celsius */
        int temp = read_temp_raw() * 100 >> 7;
        // Display the temperature in degrees Celsius, formatted to show two decimal places.
        printf("Temperature: %d.%02d °C\n", temp / 100, (temp < 0 ? -temp : temp) % 100);

        // floating point functions are also available for converting temp_result to Cesius or Fahrenheit
        //printf("\nTemperature: %.2f °C\t%.2f °F", read_temp_celsius(), read_temp_fahrenheit());
    }

    return 0;
}

// check if TMP117 is at the specified address and has correct device ID.
void check_status(void) {
    uint8_t address = tmp117_get_address();
    int status = begin();

    switch (status) {
        case TMP117_OK:
            printf("\nTMP117 found at address 0x%02X, I2C frequency %dkHz\n", address, frequency / 1000);
            break;

        case PICO_ERROR_TIMEOUT:
            printf("\nI2C timeout reached after %u microseconds\n", SMBUS_TIMEOUT_US);
            while (1) {
                tight_loop_contents();  // Halt execution if timeout occurs
            }
            break;

        case PICO_ERROR_GENERIC:
            printf("\nNo I2C device found at address 0x%02X\n", address);
            while (1) {
                tight_loop_contents();  // Halt execution if no device found
            }
            break;

        case TMP117_ID_NOT_FOUND:
            printf("\nNon-TMP117 device found at address 0x%02X\n", address);
            while (1) {
                tight_loop_contents();  // Halt execution if a wrong device is found
            }
            break;

        default:
            printf("\nUnknown error during TMP117 initialization\n");
            while (1) {
                tight_loop_contents();  // Halt execution for unexpected errors
            }
    }
}

// check if I2C is running on Pico
void check_i2c(void) {
    if (frequency == 0) {
        printf("I2C has no clock.\n");
        while(1) {
            tight_loop_contents();  // Halt execution if I2C frequency is zero
        };
    }
}