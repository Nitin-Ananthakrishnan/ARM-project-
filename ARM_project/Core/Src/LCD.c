#include "stm32f4xx.h"
#include "LCD.h"

#define I2C_ADDR  0x27 << 1      // PCF8574 address (shifted for write)
#define LCD_BACKLIGHT 0x08

#define LCD_RS 0x01
#define LCD_RW 0x02
#define LCD_EN 0x04

// ------------------- DELAY --------------------
void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 4000; i++);
}

// ------------------- I2C WRITE --------------------
void i2c_write(uint8_t data) {
    while ((I2C1->SR2 & I2C_SR2_BUSY));

    I2C1->CR1 |= I2C_CR1_START;                // START
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = I2C_ADDR;                       // Slave address + Write
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;                           // Clear ADDR

    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;                           // Write data

    while (!(I2C1->SR1 & I2C_SR1_BTF));

    I2C1->CR1 |= I2C_CR1_STOP;                 // STOP
}

// ------------------- LCD LOW-LEVEL --------------------
void lcd_pulse(uint8_t data) {
    i2c_write(data | LCD_EN | LCD_BACKLIGHT);
    delay_ms(1);
    i2c_write((data & ~LCD_EN) | LCD_BACKLIGHT);
    delay_ms(1);
}

void lcd_send_nibble(uint8_t nib, uint8_t mode) {
    uint8_t data = (nib << 4) | mode | LCD_BACKLIGHT;
    lcd_pulse(data);
}

void lcd_send_byte(uint8_t byte, uint8_t mode) {
    lcd_send_nibble(byte >> 4, mode);   // High nibble
    lcd_send_nibble(byte & 0x0F, mode); // Low nibble
}

// ------------------- LCD COMMANDS --------------------
void lcd_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, 0); // RS=0
    delay_ms(2);
}

void lcd_data(uint8_t data) {
    lcd_send_byte(data, LCD_RS);
    delay_ms(2);
}

void lcd_init(void) {
    delay_ms(50);

    // 4-bit init sequence
    lcd_send_nibble(0x03, 0);
    delay_ms(5);
    lcd_send_nibble(0x03, 0);
    delay_ms(1);
    lcd_send_nibble(0x03, 0);
    lcd_send_nibble(0x02, 0);

    lcd_cmd(0x28);  // 4-bit, 2 lines
    lcd_cmd(0x0C);  // Display ON
    lcd_cmd(0x06);  // Auto-increment
    lcd_cmd(0x01);  // Clear display
    delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
    lcd_cmd(0x80 | addr);
}

void lcd_print(char* str) {
    while (*str)
        lcd_data(*str++);
}

// ------------------- MAIN PROGRAM --------------------
//int main(void) {
//
//    // Enable GPIOB and I2C1 clock
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
//
//    // PB6 = SCL, PB7 = SDA → AF4
//    GPIOB->MODER &= ~((3 << (6*2)) | (3 << (7*2)));
//    GPIOB->MODER |=  ((2 << (6*2)) | (2 << (7*2)));
//
//    GPIOB->AFR[0] |= (4 << (6*4)) | (4 << (7*4));
//
//    // Open-drain + pull-up
//    GPIOB->OTYPER |= (1<<6) | (1<<7);
//    GPIOB->PUPDR |= (1<<(6*2)) | (1<<(7*2));  // pull-up
//
//    // I2C configuration
//    I2C1->CR2 = 16;      // APB1 = 16 MHz
//    I2C1->CCR = 80;      // 100 kHz
//    I2C1->TRISE = 17;
//    I2C1->CR1 |= I2C_CR1_PE;
//
//    lcd_init();
//
//    lcd_set_cursor(0, 0);
//    lcd_print("Hello I2C LCD");
//
//    lcd_set_cursor(1, 0);
//    lcd_print("Bare Metal I2C");
//
//    while (1) { }
//}
