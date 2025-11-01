#include "i2c-lcd.h"

// Store the I2C handle
static I2C_HandleTypeDef *g_hi2c;

/**
 * @brief Writes a 4-bit value to the LCD.
 * @param nibble The 4-bit value.
 * @param is_data 1 if sending data, 0 if sending command.
 */
static void lcd_write_nibble(uint8_t nibble, uint8_t is_data) {
    // The PCF8574 controls the LCD pins. A common mapping is:
    // P0: RS, P1: RW, P2: E, P3: Backlight
    // P4: D4, P5: D5, P6: D6, P7: D7
    uint8_t data = (nibble << 4) | (is_data ? 0x01 : 0x00) | 0x08; // Set RS, Backlight ON
    uint8_t buffer[2];

    // Pulse the enable (E) pin, which is connected to P2
    buffer[0] = data | 0x04;  // Set E high
    buffer[1] = data & ~0x04; // Set E low

    HAL_I2C_Master_Transmit(g_hi2c, LCD_I2C_ADDRESS, buffer, 2, HAL_MAX_DELAY);
    HAL_Delay(1);
}

void lcd_send_cmd(char cmd) {
    uint8_t high_nibble = cmd >> 4;
    uint8_t low_nibble = cmd & 0x0F;

    lcd_write_nibble(high_nibble, 0); // 0 for command
    lcd_write_nibble(low_nibble, 0);
}

void lcd_send_data(char data) {
    uint8_t high_nibble = data >> 4;
    uint8_t low_nibble = data & 0x0F;

    lcd_write_nibble(high_nibble, 1); // 1 for data
    lcd_write_nibble(low_nibble, 1);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01); // Clear display command
    HAL_Delay(2);
}

void lcd_put_cur(int row, int col) {
    uint8_t address;
    switch (row) {
        case 0:
            address = 0x80 + col;
            break;
        case 1:
            address = 0xC0 + col;
            break;
        default:
            address = 0x80 + col;
            break;
    }
    lcd_send_cmd(address);
}

void lcd_init(I2C_HandleTypeDef *hi2c) {
    g_hi2c = hi2c;

    // Initialization sequence for 4-bit mode
    HAL_Delay(50);
    lcd_write_nibble(0x03, 0);
    HAL_Delay(5);
    lcd_write_nibble(0x03, 0);
    HAL_Delay(1);
    lcd_write_nibble(0x03, 0);
    HAL_Delay(1);

    // Set to 4-bit mode
    lcd_write_nibble(0x02, 0);

    // Now send full commands
    lcd_send_cmd(0x28); // Function set: 4-bit mode, 2 lines, 5x8 dots
    lcd_send_cmd(0x0C); // Display on, cursor off, blink off
    lcd_send_cmd(0x06); // Entry mode set: increment cursor, no shift
    lcd_clear();       // Clear display
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}
