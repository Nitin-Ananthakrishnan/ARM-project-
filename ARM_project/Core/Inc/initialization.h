/*
 * initialization.h
 *
 *  Created on: Nov 7, 2025
 *      Author: nitin
 */

#ifndef INC_INITIALIZATION_H_
#define INC_INITIALIZATION_H_

extern volatile bool toggle_switch1;
extern volatile bool toggle_switch2;
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
