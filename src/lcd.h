#include "pico/stdlib.h"
#include <stdint.h>

// GPIO mapping
#define LCD_RS  10   // Register Select
#define LCD_EN  11   // Enable

#define LCD_D4  18   // Data pin 4
#define LCD_D5  19   // Data pin 5
#define LCD_D6  20   // Data pin 6
#define LCD_D7  21   // Data pin 7

void lcd_gpio_init(void);
void lcd_pulse_enable(void);
void lcd_send_nibble(uint8_t nibble);
void lcd_send_byte(uint8_t rs, uint8_t data);
void lcd_init(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char *str);