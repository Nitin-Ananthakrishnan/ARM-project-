/*
 * Screen.c
 *
 *  Created on: Nov 8, 2025
 *      Author: nitin
 */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "Screen.h"

void home_screen(){
	         HD44780_Backlight();
	         HD44780_Clear();
	         HD44780_SetCursor(0,0);
	    	 HD44780_PrintStr("1. Drip");
	    	 HD44780_SetCursor(0,1);
	    	 HD44780_PrintStr("2. Fan");
	    	 HD44780_SetCursor(9,1);
	    	 HD44780_PrintStr("3.Sleep");
	    	 timer(500);
}
