# TMP117 Library for Raspberry Pi Pico SDK

This version is designed as an **INTERFACE** library for the Raspberry Pi Pico SDK.

Based on the **SparkFun TMP117 Arduino Library** by Madison Chodikov, this library has been ported to the Raspberry Pi Pico C SDK. While the basics of the original library remain, much has changed in this version. It was developed for personal use with the TMP117 sensor and as an opportunity to learn the Pico SDK.

## Test boards
- **Raspberry Pi Pico H (RP2040)**
- **Pimoroni Pico Plus 2 (RP2350)**
- **SparkFun SEN-15805 (TMP117)**

### Reference Materials
- **Texas Instruments TMP117 High-Accuracy, Low-Power Digital Temperature Sensor**  
  Datasheet: [TMP117 Datasheet](https://www.ti.com/lit/gpn/TMP117)
- **SparkFun TMP117 Board**: [SparkFun SEN-15805](https://www.sparkfun.com/products/15805)
- **Raspberry Pi Pico Documentation**: [Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/)
- **Raspberry Pi Pico SDK**: [Pico SDK](https://github.com/raspberrypi/pico-sdk)

---

## Library Notes

### Examples
- **13 Examples** are provided, some within the same directory.
- Tested successfully with **Pico SDK 2.1.0** for both RP2040 and RP2350 boards.
- Default board file I2C and UART pins are used. Configure the alert interrupt pin in examples if needed.

In the **first temperature reading example (`temp_result.c`)**, Q notation is introduced to display temperature values without using floating-point math.  

Refer to the **TI Application Note: How to Read and Interpret Digital Temperature Sensor Output Data**:  
"Modern sensors, such as the TMP117, offer a full 16 bits of resolution in a Q7 format."

The **16-bit Word** of the `temp_result` register is structured as follows:
- **Bit 15**: Sign bit  **Bits [14:7]**: Integer bits  **"Decimal Point"**  **Bits [6:0]**: Fractional bits  

---

### Key Features
1. **Defined Types and Enumerations**: Many function parameters now use types and enumerations for clarity. See `tmp117.h` for details.
2. **Efficient Register Reads**: The `read_register` function tracks the last pointer register written to. This avoids unnecessary writes to the pointer register.
3. **EEPROM Writes**: Writes to registers are **volatile** unless EEPROM programming is used.
4. **New Functions and Examples**:
   - Raw temperature register read for Q notation display.
   - one-shot mode examples
   - Alert pin usage as a Data Ready interrupt
   - SMBus alert response example.
   - EEPROM programming functions and examples.

### Handling Flags and One-Shot Mode
- Reading the **configuration register** clears the **high alert** and **low alert** flags.
- Reading the **temperature result** or **configuration register** clears the **data ready flag**.
- In **one-shot mode**, ensure the temperature register is not read before the configuration register to avoid prematurely clearing the data ready flag.

### Factory defaults and EEPROM
- Factory default: **8 averaged temperature conversions, conversion cycle time 1 second in CC mode**.
- See table **Table 7-7. Conversion Cycle Time in CC Mode** in the datasheet for details.
- Averaging is supported in both continuous and one-shot modes.  
  See section **7.3.2 Averaging** in the datasheet for details.
- **EEPROM Write Limits**: Typical lifespan is **50,000 writes** (minimum **1,000 writes**).  
  Consider recording the values of the general-purpose EEPROM before overwriting them.

---

## I2C Details
- Ensure your I2C board has **pullups**, or add them, probably in the 2.2k-4.7k ohm range.
- Possible TMP117 I2C addresses: **0x48, 0x49, 0x4A, 0x4B** (set via jumpers on the SparkFun board).  
  Default address is **0x48** unless changed using the `set_address()` function.
- Use `tmp117_set_instance()` to easily change I2C instance instead of using a board file, if desired.

**Note**: The device requires **1.5 ms** to power up before conversions can begin.

---

## File Structure
- `/src` - Library source and header files.
- `/examples` - Example applications for testing and demonstration.

---

## Acknowledgments
Thank you to Madison Chodikov @ SparkFun Electronics for writing the original SparkFun TMP117 Qwiic Arduino library and board design. Thanks to kilograham and all contributors to the Raspberry Pi Pico SDK.

See `LICENSE.txt` for licensing details.
