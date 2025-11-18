#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "Functions.h"

#define READ_PIN(GPIOx, PIN)   ((GPIOx->IDR & (1 << PIN)) ? 1 : 0)

void ten_or_fifteen_mins_timer(uint8_t a){
	char temp[16];
	if (a==10){
		for(uint8_t i=10;i>0;i--){
			 HD44780_Clear();
			 HD44780_SetCursor(0,0);
			 HD44780_PrintStr("Time left:");
			 sprintf(temp,"%d",i);
			 HD44780_SetCursor(11,0);
		     HD44780_PrintStr(temp);
		     timer(65535);

		}
	}
	if (a==15){
			for(uint8_t i=15;i>0;i--){
				 HD44780_Clear();
				 HD44780_SetCursor(0,0);
				 HD44780_PrintStr("Time left:");
				 //sprintf(temp,"%d",i);
				 HD44780_SetCursor(11,0);
			     HD44780_PrintStr(temp);
			     timer(65535);

			}
		}
}
bool BtnDebounce(GPIO_TypeDef* GPIOx, uint8_t PinNumber) {
	uint16_t samples = 0;
	    for (int i = 0; i < 16; i++) {
	        samples = (samples << 1) | READ_PIN(GPIOx, PinNumber);
	    }
	    return (samples == 0xFFFF);   // stable high for 16 readsads
}
void timer_arr(uint32_t delay)
  {
   while(delay>0)
   {
	 uint32_t temp=0;
	  if(delay>65535)
	  {
		  temp=65535;

	  }
	  else
	  {
		  temp=delay;

	  }
	  TIM3->PSC=16000-1;
	  TIM3->ARR=temp-1;
	  TIM3->CNT=0;
	  TIM3->EGR=TIM_EGR_UG;
	  TIM3->SR=0x0000;
	  TIM3->CR1=TIM_CR1_CEN;
	  while(!(TIM3->SR&(0X0001)));
	  TIM3->SR|=0x0000;
	  TIM3->CR1|=0x0000;
      delay=delay-temp;

   }
  }
/**
 * @brief The timer() is used to call the time delay function
 * @param a the time delay in milli-seconds to be created
 */
void timer(uint32_t a){
	  RCC->APB1ENR|=0X00000002;
	  TIM3->CR1|=0x0000;
	  timer_arr(a);

}
