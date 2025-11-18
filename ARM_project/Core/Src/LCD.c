#include "stm32f4xx.h"
#include "LCD.h"
#include "stdbool.h"
#define I2C_ADDR  0x27 << 1      // PCF8574 address (shifted for write)
#define LCD_BACKLIGHT 0x08

#define LCD_RS 0x01
#define LCD_RW 0x02
#define LCD_EN 0x04


typedef enum {
    MENU_IDLE,
    MENU_MAIN,
    MENU_DRIP,
    MENU_FAN
} MenuState;

volatile uint8_t display_on = false;
MenuState menu_state = MENU_IDLE;

uint8_t drip_time = 10;      // 10 or 15
uint8_t fan_mode = 0;        // 0 = auto, 1 = manual

// ------------------ BUTTON READ -------------------
uint8_t read_btn_pa4() { return (GPIOA->IDR & (1<<4)) != 0; }
uint8_t read_btn_pa5() { return (GPIOA->IDR & (1<<5)) != 0; }
volatile uint8_t last_btn_pa4 = 0;
volatile uint8_t last_btn_pa5 = 0;
// ------------------- DELAY --------------------
uint8_t btn_pa4_pressed() {
    uint8_t state = (GPIOA->IDR & (1<<4)) != 0;
    if (state && !last_btn_pa4) { // rising edge
        last_btn_pa4 = 1;
        return 1;
    }
    last_btn_pa4 = state;
    return 0;
}

uint8_t btn_pa5_pressed() {
    uint8_t state = (GPIOA->IDR & (1<<5)) != 0;
    if (state && !last_btn_pa5) {
        last_btn_pa5 = 1;
        return 1;
    }
    last_btn_pa5 = state;
    return 0;
}
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
void menu_delay(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms*4000; i++);
}
void menu_show_main() {
    lcd_cmd(0x01);
    lcd_set_cursor(0,0);
    lcd_print("Select Function:");
    lcd_set_cursor(1,0);
    lcd_print("1)Drip  2)Fan");
}

void menu_show_drip() {
    lcd_cmd(0x01);
    lcd_set_cursor(0,0);
    lcd_print("Set Drip Time:");
    lcd_set_cursor(1,0);
    lcd_print("1)10m  2)15m");
}

void menu_show_fan() {
    lcd_cmd(0x01);
    lcd_set_cursor(0,0);
    lcd_print("Fan Mode:");
    lcd_set_cursor(1,0);
    lcd_print("1)Auto 2)Manual");
}

void menu_show_message(const char *msg) {
    lcd_cmd(0x01);
    lcd_set_cursor(0,0);
    lcd_print(msg);
}
void process_menu() {

    if (!display_on) {
        menu_state = MENU_IDLE;
        return;
    }

    if (menu_state == MENU_IDLE) {
        menu_state = MENU_MAIN;
        menu_show_main();
    }
    if (menu_state == MENU_MAIN) {
        if (btn_pa4_pressed()) {
            menu_state = MENU_DRIP;
            menu_delay(300);
            menu_show_drip();
        }
        if (btn_pa5_pressed()) {
            menu_state = MENU_FAN;
            menu_delay(300);
            menu_show_fan();
        }
        return;
    }
    if (menu_state == MENU_DRIP) {
        if (btn_pa4_pressed()) {
            drip_time = 10;
            menu_show_message("Drip set 10 min");
            menu_delay(2000);
            menu_state = MENU_IDLE;
            return;
        }
        if (btn_pa5_pressed()) {
            drip_time = 15;
            menu_show_message("Drip set 15 min");
            menu_delay(2000);
            menu_state = MENU_IDLE;
            return;
        }
        return;
    }
    if (menu_state == MENU_FAN) {
        if (btn_pa4_pressed()) {
            fan_mode = 0; // auto
            menu_show_message("Fan set Auto");
            menu_delay(2000);
            menu_state = MENU_IDLE;
            return;
        }
        if (btn_pa5_pressed()) {
            fan_mode = 1; // manual
            menu_show_message("Fan Manual");
            menu_delay(2000);
            menu_state = MENU_IDLE;
            return;
        }
        return;
    }
}
