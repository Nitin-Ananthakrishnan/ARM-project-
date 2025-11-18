/*
 * LCD.h
 *
 *  Created on: Nov 17, 2025
 *      Author: nitin
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

uint8_t read_btn_pa5();
uint8_t read_btn_pa4() ;
uint8_t btn_pa4_pressed();
uint8_t btn_pa5_pressed();

void delay_ms(uint32_t ms);
void i2c_write(uint8_t data) ;
void lcd_pulse(uint8_t data) ;
void lcd_send_nibble(uint8_t nib, uint8_t mode);
void lcd_send_byte(uint8_t byte, uint8_t mode) ;
extern void lcd_cmd(uint8_t cmd) ;
void lcd_data(uint8_t data);
void lcd_init(void);
extern void lcd_set_cursor(uint8_t row, uint8_t col);
extern void lcd_print(char* str);
void menu_delay(uint32_t ms);
extern void menu_show_main() ;
void menu_show_drip() ;
void menu_show_fan();
void menu_show_message(const char *msg);
void process_menu();



#endif /* INC_LCD_H_ */
