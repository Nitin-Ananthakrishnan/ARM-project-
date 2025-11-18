/*
 * LCD.h
 *
 *  Created on: Nov 17, 2025
 *      Author: nitin
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

void delay_ms(uint32_t ms);
void i2c_write(uint8_t data) ;
void lcd_pulse(uint8_t data) ;
void lcd_send_nibble(uint8_t nib, uint8_t mode);
void lcd_send_byte(uint8_t byte, uint8_t mode) ;
void lcd_cmd(uint8_t cmd) ;
void lcd_data(uint8_t data);
void lcd_init(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(char* str);



#endif /* INC_LCD_H_ */
