// temp_lcd.c
/**
 * Copyright (c) 2024 breaker
 * Based on code from:
 * - Raspberry Pi Pico SDK (c) 2020 Raspberry Pi (Trading) Ltd. (BSD-3-Clause License)
 * - SparkFun TMP117 Arduino Library (c) 2016 SparkFun Electronics (originally MIT, relicensed BSD-3-Clause)
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// HD44780 16x2 character LCD display version

// Original SparkFun TMP117 Arduino Library file: Example1_BasicReadings.ino

/* Notes for this new version using the Raspberry Pi Pico/RP2040:
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

// GPIO mapping for LCD is in src/lcd.h - edit that file to change pins for LCD drive
#include "lcd.h"
#include "tmp117.h"
#include "tmp117_registers.h"
#include "pico/stdlib.h"
#include "pico/printf.h"
#include "hardware/i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SERIAL_INIT_DELAY_MS 1000 // adjust as needed to mitigate garbage characters after serial interface is started
#define TMP117_I2C_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN // set to a different SDA pin as needed
#define TMP117_I2C_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN // set to a different SCL pin as needed
#define TMP117_OFFSET_VALUE -25.0f  // temperature offset in degrees C set by user (try negative values for testing)
#define TMP117_CONVERSION_DELAY_MS 1000 // Adjust the delay based on conversion cycle time and preference

void check_i2c(void);
void check_status(void);
void lcd_msg(void);

// variable for checking I2C frequency
uint frequency = 0;

int main(void) {
    // initialize chosen interface
    stdio_init_all();
    // a little delay to ensure serial line stability
    sleep_ms(SERIAL_INIT_DELAY_MS);

    printf("Temperature readings on 16x2 character LCD\n");

    lcd_msg();

    // uncomment below to set I2C address other than 0x48 (e.g., 0x49)
    //tmp117_set_address(0x49);

    // Selects I2C instance (i2c_default is set as default in the tmp117.c)
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
           2) Multiply by 100 to scale the temperature (i.e. 2 decimal places)
           3) Shift right by 7 to account for the TMP117's 1/128 resolution (Q7 format) */
        int temp = read_temp_raw() * 100 >> 7;
        int integer = temp / 100;
        int decimal = (temp < 0 ? -temp : temp) % 100;
        char buffer[8] = {0}; // Buffer to hold the formatted temperature string

        // Format the temperature as "integer.decimal"
        sprintf(buffer, "%d.%02d", integer, decimal); // Ensure two decimal places

        lcd_set_cursor(1, 0);  // Row 1, Col 0
        // Print the formatted temperature string to the LCD
        lcd_print(buffer);

        // Display the temperature in degrees Celsius to the serial monitor
        printf("Temperature: %d.%02d °C\n", integer, decimal);

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
            printf("TMP117 found at address 0x%02X, I2C frequency %dkHz\n", address, frequency / 1000);
            break;

        case PICO_ERROR_TIMEOUT:
            printf("I2C timeout reached after %u microseconds\n", SMBUS_TIMEOUT_US);
            while (1) {
                tight_loop_contents();  // Halt execution if timeout occurs
            }
            break;

        case PICO_ERROR_GENERIC:
            printf("No I2C device found at address 0x%02X\n", address);
            while (1) {
                tight_loop_contents();  // Halt execution if no device found
            }
            break;

        case TMP117_ID_NOT_FOUND:
            printf("Non-TMP117 device found at address 0x%02X\n", address);
            while (1) {
                tight_loop_contents();  // Halt execution if a wrong device is found
            }
            break;

        default:
            printf("Unknown error during TMP117 initialization\n");
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

// LCD init and display characters that never change
void lcd_msg(void) {
    lcd_init();
    lcd_set_cursor(0, 0);  // Row 0, Col 0
    lcd_print("Temperature:");
    lcd_set_cursor(1, 7);  // Row 0, Col 7
    lcd_print("degrees C");
}
