////#include "stm32f4xx.h"
////#include "initialization.h"
////volatile char rx_buffer[32];
////volatile uint8_t rx_index = 0;
////
////volatile uint8_t drip10_trigger = 0;
////
////void usart1_write_char(char c) {
////    while (!(USART1->SR & USART_SR_TXE)) { }
////    USART1->DR = (uint16_t)c;}
//// void usart1_write_str(const char *s) {
////    while (*s) usart1_write_char(*s++);
////}
////char usart1_read_char_blocking(void) {
////    while (!(USART1->SR & USART_SR_RXNE)) { }
////    return (char)(USART1->DR);}
////void usart1_read_line(char *buf, uint32_t maxlen) {
////    uint32_t i = 0;
////    while (i < maxlen - 1) {
////        char c = usart1_read_char_blocking();
////        if (c == '\r') {
////            char n = usart1_read_char_blocking();
////            if (n != '\n') {
////                if (i < maxlen - 1) buf[i++] = n;
////            }
////            break;
////        }
////        if (c == '\n') break;
////        buf[i++] = c;
////    }
////    buf[i] = '\0';
////}
////void gpio_usart_init(void) {
////    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA clock
////
////    GPIOA->MODER &= ~((3U << (9 * 2)) | (3U << (10 * 2)));
////    GPIOA->MODER |=  ((2U << (9 * 2)) | (2U << (10 * 2))); // 10 = AF
////
////    GPIOA->AFR[1]&=~((0x7<<8)|(0x7<<4));
////    GPIOA->AFR[1]|=(0x7<<8)|(0x7<<4);
////
////    // RX pull-up helps hold idle-high if line is floating during wiring
////   GPIOA->PUPDR &= ~(3U << (10 * 2));
////   GPIOA->PUPDR |=  (1U << (10 * 2)); // pull-up on RX
////}
////
////// ------------------- Init USART1 115200-8-N-1 -----------------------
////void usart1_init(void) {
////    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;    // USART1 clock
////
////    USART1->CR1 = 0;                         // ensure disabled
////    // Baud = 115200 @ PCLK2=84MHz, oversampling=16 -> BRR = 0x2D93
////    USART1->BRR = 0x2D93;
////
////    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; // enable TX, RX
////    USART1->CR1 |= USART_CR1_RXNEIE;
////    USART1->CR1 |= USART_CR1_UE;  // USART enable
////
////    USART1->CR1 &= ~USART_CR1_PS;  // No parity
////    USART1->CR1 &= ~USART_CR1_M;   // 8 data bits
////
////    NVIC_SetPriority(USART1_IRQn, 0);
////    NVIC_EnableIRQ(USART1_IRQn);
////}
////
//////// ------------------- Main ------------------------------------------
//////
//////
//////    // Announce readiness once
//////    usart1_write_str("Ready to receive commands\n");
//////
//////    char cmd[32];
//////    while (1) {
//////        usart1_read_line(cmd, sizeof(cmd));
//////
//////        if (!cmd[0]) continue;
//////
//////        if (0 == strcmp(cmd, "Led ON")) {
//////            led_on();
//////            usart1_write_str("Led is ON\n");
//////        } else if (0 == strcmp(cmd, "Led OFF")) {
//////            led_off();
//////            usart1_write_str("Led is OFF\n");
//////        } else if (0 == strcmp(cmd, "Status")) {
//////            if (led_is_on()) usart1_write_str("Led is ON\n");
//////            else             usart1_write_str("Led is OFF\n");
//////        } else {
//////            usart1_write_str("Unknown command\n");
//////        }
//////    }
//////}
////void USART1_IRQHandler(void)
////{
////    if (USART1->SR & USART_SR_RXNE)   // byte received?
////    {
////        char c = USART1->DR;          // read byte
////        // Handle CR (\r) so it does not break matching
////		if (c == '\r')  return;
////
////        if (c != '\n' && rx_index < sizeof(rx_buffer) - 1)
////        {
////            rx_buffer[rx_index++] = c;
////        }
////        else
////        {
////            rx_buffer[rx_index] = '\0';
////            rx_index = 0;
////
////            // ---------- MATCH COMMAND HERE ----------
////            if (strcmp((char*)rx_buffer, "Drip 10") == 0)
////            {
////               GPIOA->ODR^=(1<<5);
////            }
////            if (strcmp((char*)rx_buffer, "Drip 15") == 0)
////             {
////                           // drip10_trigger = 1;   // <-- your interrupt trigger
////			}
////        }
////    }
////}
////
////////////
//#include "stm32f4xx.h"
//#include <string.h>
//
//#define RX_BUFFER_SIZE 32
//
//volatile char rx_buffer[RX_BUFFER_SIZE];
//volatile uint8_t rx_index = 0;
//
//// Flags for main loop
//volatile uint8_t event_drip10 = 0;
//volatile uint8_t event_drip15 = 0;
//
//void USART1_Init(void)
//{
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
//    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
//
//    // PA9 / PA10 = AF7
//    GPIOA->MODER &= ~(0xF << (9 * 2));
//    GPIOA->MODER |=  (0xA << (9 * 2));
//
//    GPIOA->AFR[1] &= ~((0xF << 4) | (0xF << 8));  // Clear AF bits for PA9/PA10
//    GPIOA->AFR[1] |=  (7 << 4) | (7 << 8);        // Set AF7
//
//
//    USART1->BRR = 0x2D9B;
//    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
//    USART1->CR1 |= USART_CR1_UE;
//
//    NVIC_EnableIRQ(USART1_IRQn);
//}
//
//// --------------------------------
//// Transmit
//// --------------------------------
//void USART1_WriteChar(char c)
//{
//    while (!(USART1->SR & USART_SR_TXE));
//    USART1->DR = c;
//}
//
//void USART1_WriteString(const char *s)
//{
//    while (*s)
//        USART1_WriteChar(*s++);
//}
//
//void USART1_WriteLine(const char *s)
//{
//    USART1_WriteString(s);
//    USART1_WriteString("\r\n");
//}
//
//// --------------------------------
//// Receive interrupt
//// --------------------------------
//void USART1_IRQHandler(void)
//{
//    if (USART1->SR & USART_SR_RXNE)
//    {
//        char c = USART1->DR;
//
//        if (c == '\n' || c == '\r')
//        {
//            rx_buffer[rx_index] = 0;
//
//            if (strcmp(rx_buffer, "Drip 10") == 0)
//                event_drip10 = 1;
//
//            else if (strcmp(rx_buffer, "Drip 15") == 0)
//                event_drip15 = 1;
//
//            rx_index = 0;
//        }
//        else
//        {
//            if (rx_index < RX_BUFFER_SIZE - 1)
//                rx_buffer[rx_index++] = c;
//        }
//    }
//}
