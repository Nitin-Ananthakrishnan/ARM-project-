/*
 * Functions.h
 *
 *  Created on: Nov 8, 2025
 *      Author: nitin
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

void ten_or_fifteen_mins_timer(uint8_t a);
bool BtnDebounce(GPIO_TypeDef* GPIOx, uint8_t PinNumber) ;
void timer(uint32_t a);
void timer_arr(uint32_t delay);


#endif /* INC_FUNCTIONS_H_ */
