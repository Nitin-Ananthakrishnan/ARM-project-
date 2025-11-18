/*
 * initialization.h
 *
 *  Created on: Nov 7, 2025
 *      Author: nitin
 */
#ifndef INC_INITIALIZATION_H_
#define INC_INITIALIZATION_H_
#include <stdint.h>
#include <stdbool.h>
extern volatile bool DRIP_TOGGLE_PIN;
extern volatile bool FAN_TOGGLE_PIN;
extern volatile bool display_on;

void GPIO_configuration(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void timer_init();
void RTC_configuration(uint8_t hours,uint8_t minutes,uint8_t seconds);
void RTC_intrupt(uint8_t hours, uint8_t minutes, uint8_t seconds);
void EXTI3_IRQHandler(void);
void test_led();

#endif /* INC_INITIALIZATION_H_ */
