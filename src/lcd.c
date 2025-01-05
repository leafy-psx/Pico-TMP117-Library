
#include "lcd.h"
#include "pico/stdlib.h"
#include <stdint.h>

void lcd_gpio_init(void) {
    gpio_init(LCD_RS);
    gpio_set_dir(LCD_RS, GPIO_OUT);
    gpio_init(LCD_EN);
    gpio_set_dir(LCD_EN, GPIO_OUT);

    gpio_init(LCD_D4);
    gpio_set_dir(LCD_D4, GPIO_OUT);
    gpio_init(LCD_D5);
    gpio_set_dir(LCD_D5, GPIO_OUT);
    gpio_init(LCD_D6);
    gpio_set_dir(LCD_D6, GPIO_OUT);
    gpio_init(LCD_D7);
    gpio_set_dir(LCD_D7, GPIO_OUT);
}

void lcd_pulse_enable(void) {
    gpio_put(LCD_EN, 1);
    sleep_us(1);  // Enable pulse width > 450ns
    gpio_put(LCD_EN, 0);
    sleep_us(50); // Commands need > 37µs to settle
}

void lcd_send_nibble(uint8_t nibble) {
    gpio_put(LCD_D4, (nibble >> 0) & 1);
    gpio_put(LCD_D5, (nibble >> 1) & 1);
    gpio_put(LCD_D6, (nibble >> 2) & 1);
    gpio_put(LCD_D7, (nibble >> 3) & 1);
    lcd_pulse_enable();
}

void lcd_send_byte(uint8_t rs, uint8_t data) {
    gpio_put(LCD_RS, rs);  // RS=0 for command, 1 for data
    lcd_send_nibble(data >> 4); // Send higher nibble
    lcd_send_nibble(data & 0xF); // Send lower nibble
    sleep_us(50); // Wait for the command to execute
}

void lcd_init(void) {
    lcd_gpio_init();
    sleep_ms(15); // Wait for LCD to power up

    // Initialization sequence
    lcd_send_nibble(0x3); // Function set in 8-bit mode
    sleep_ms(5);
    lcd_send_nibble(0x3);
    sleep_us(150);
    lcd_send_nibble(0x3);
    sleep_us(50);
    lcd_send_nibble(0x2); // Switch to 4-bit mode

    // Function set: 4-bit mode, 2 lines, 5x8 font
    lcd_send_byte(0, 0x28);
    // Display ON/OFF control: Display ON, Cursor OFF, Blink OFF
    lcd_send_byte(0, 0x0C);
    // Clear display
    lcd_send_byte(0, 0x01);
    sleep_ms(2);
    // Entry mode set: Increment cursor, No shift
    lcd_send_byte(0, 0x06);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0 ? 0x00 : 0x40) + col;
    lcd_send_byte(0, 0x80 | address); // Set DDRAM address
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_send_byte(1, *str++);
    }
}