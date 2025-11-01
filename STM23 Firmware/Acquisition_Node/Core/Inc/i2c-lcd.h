#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "main.h"

// Define the I2C address of the LCD module.
// This is typically 0x27 or 0x3F for PCF8574-based modules.
// You may need to use an I2C scanner to find the correct address.
#define LCD_I2C_ADDRESS (0x27 << 1) // Shift left by 1 for HAL library

/**
 * @brief Initialize the LCD module.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for I2C2.
 */
void lcd_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Send a command to the LCD.
 * @param cmd The command byte to send.
 */
void lcd_send_cmd(char cmd);

/**
 * @brief Send a data character to the LCD.
 * @param data The character to display.
 */
void lcd_send_data(char data);

/**
 * @brief Send a string to be displayed on the LCD.
 * @param str The null-terminated string to send.
 */
void lcd_send_string(char *str);

/**
 * @brief Position the cursor on the LCD.
 * @param row The row number (0 or 1 for 16x2 LCD).
 * @param col The column number (0-15 for 16x2 LCD).
 */
void lcd_put_cur(int row, int col);

/**
 * @brief Clear the LCD screen.
 */
void lcd_clear(void);

#endif // I2C_LCD_H
