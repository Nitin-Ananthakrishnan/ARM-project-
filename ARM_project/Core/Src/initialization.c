#include "stm32f4xx.h"
//#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "initialization.h"  // Include the header for the declaration

volatile bool toggle_switch1=0;
volatile bool toggle_switch2=0;
volatile bool display_on=0;

#define BIN2BCD(x) (((x / 10) << 4) | (x % 10))

void test_led(){
	GPIOA->MODER |= GPIO_MODER_MODE5_0; //for inbuit led
	GPIOA->OTYPER &= ~(1 << 5);
	GPIOA->OSPEEDR &= ~(0x3 << (2 * 5));
	GPIOA->PUPDR &= ~(0x3 << (2 * 5));
}
void GPIO_configuration(void)
{
	//only for switches
	GPIOA->MODER = 0x00000000;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// Configure PA0 and PA1 as input for toggle switches
	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1|GPIO_MODER_MODE2|GPIO_MODER_MODE3);  // Input mode for PA0 and PA1

	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;  // Pull-down for PA0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1;  // Pull-down for PA1
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_1;  // Pull-down for PA2
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_1;  // Pull-down for PA3

	// Configure PA2, PA3, PA4, PA5 as input for push-button switches
//	GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5); // Input mode
//	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1;  // Pull-down for PA4
//	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_1;  // Pull-down for PA5
//
//	SYSCFG->EXTICR[0] &= ~((0xF << 8)|(0xF << 4)|(0xF << 0)|(0xF<<12));
//	SYSCFG->EXTICR[0] |=  (0x0 << 8)|(0x0 << 4)|(0x0 << 0)|(0x0<<12);

	SYSCFG->EXTICR[0] &= ~(
	    (0xF << 0)  |  // Clear EXTI0 bits
	    (0xF << 4)  |  // Clear EXTI1 bits
	    (0xF << 8)  |  // Clear EXTI2 bits
	    (0xF << 12)    // Clear EXTI3 bits
	);
	// Set EXTI lines 0 to 3 to 0 for GPIOA
	SYSCFG->EXTICR[0] |= (
	    (0x0 << 0)  |  // EXTI0 -> PA0
	    (0x0 << 4)  |  // EXTI1 -> PA1
	    (0x0 << 8)  |  // EXTI2 -> PA2
	    (0x0 << 12)    // EXTI3 -> PA3
	);
	//Intrupt configuration starts here
	// Configure EXTI0 for PA0
	EXTI->IMR |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
	EXTI->FTSR |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
	EXTI->RTSR &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3));
	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);  // Enable EXTI0 interrupt for PA0
	NVIC_SetPriority(EXTI1_IRQn, 1);
	NVIC_EnableIRQ(EXTI1_IRQn);  // Enable EXTI1 interrupt for PA1
	NVIC_SetPriority(EXTI2_IRQn, 1);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_SetPriority(EXTI3_IRQn, 1);
	NVIC_EnableIRQ(EXTI3_IRQn);
}
void EXTI0_IRQHandler(void) {
    // Check if interrupt occurred on EXTI0 (PA0)
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;  // Clear the interrupt pending bit for EXTI0
        // Toggle action for PA0 (e.g., LED or a variable)
        toggle_switch2^=1;  // Assume you have a function to toggle an LED or change state
        GPIOA->ODR^=(1<<(5));
        timer(100);
    }
}

void EXTI1_IRQHandler(void) {
    // Check if interrupt occurred on EXTI1 (PA1)
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;  // Clear the interrupt pending bit for EXTI
        // Toggle action for PA1 (e.g., LED or a variable)
        toggle_switch1^=1;
        GPIOA->ODR^=(1<<(5));
        timer(100);
    }
}
void EXTI2_IRQHandler(void) {
	if (EXTI->PR & EXTI_PR_PR2) {
	        EXTI->PR = EXTI_PR_PR2;  // Clear the interrupt pending bit for EXTI
	        // Toggle action for PA1 (e.g., LED or a variable)
	        NVIC_SystemReset();
	        timer(100);
	    }

}
void EXTI3_IRQHandler(void){
	if (EXTI->PR & EXTI_PR_PR3) {
		        EXTI->PR=EXTI_PR_PR3;  // Clear the interrupt pending bit for EXTI
		        display_on^=1;
		        GPIOA->ODR^=(1<<(5));
		        timer(100);
		    }

}

void timer_init(){
	 RCC->AHB1ENR|=0X00000001;
	 GPIOA->MODER|=(1<<10);
	 GPIOA->OTYPER|=0X00000000;
	 GPIOA->OSPEEDR|=0X00000000;
	 GPIOA->PUPDR|=0X00000000;
}

void RTC_configuration(uint8_t hours,uint8_t minutes,uint8_t seconds){
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |=(1<<8);
	RCC->CSR |= RCC_CSR_LSION;
	while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }  // Wait until LSI is ready
//	RCC->BDCR &= ~RCC_BDCR_RTCEN;
//	RTC->CR &= ~RTC_CR_FMT;
	RCC->BDCR &= ~RCC_BDCR_RTCSEL;      // Clear RTCSEL bits
	RCC->BDCR |= RCC_BDCR_RTCSEL_1;     // 10 = LSI
	RCC->BDCR |= RCC_BDCR_RTCEN;
	RTC->WPR = 0xCA;       // Disable write protection
	RTC->WPR = 0x53;
	RTC->ISR |= RTC_ISR_INIT;
	while (!(RTC->ISR & RTC_ISR_INITF)) { }
	RTC->PRER = (127 << 16) | 249;
	RTC->TR = (BIN2BCD(hours) << 16) | (BIN2BCD(minutes) << 8) | BIN2BCD(seconds);
	RTC->ISR &= ~RTC_ISR_INIT;            // Exit initialization mode
	RTC->WPR = 0xFF;
	//PWR->CR &= ~PWR_CR_DBP;  // Disable write access
}
void RTC_intrupt(uint8_t hours, uint8_t minutes, uint8_t seconds) {
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	PWR->CR |= PWR_CR_DBP;
	RTC->CR &= ~RTC_CR_ALRAE;
	while (!(RTC->ISR & RTC_ISR_ALRAWF));  // Wait until writable
	RTC->CR &= ~RTC_CR_FMT;
	RTC->ALRMAR = (BIN2BCD(hours) << 16) | (BIN2BCD(minutes) << 8) | BIN2BCD(seconds);
	RTC->ALRMAR &= ~(RTC_ALRMAR_MSK1 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK3);
	RTC->ALRMAR |= RTC_ALRMAR_MSK4;
	RTC->ISR &= ~RTC_ISR_ALRAF_Msk;
	RTC->CR |= RTC_CR_ALRAIE | RTC_CR_ALRAE;
	EXTI->IMR  |= EXTI_IMR_IM17;   // Unmask interrupt line 17
	EXTI->FTSR |= EXTI_FTSR_TR17;  // Rising edge trigger
	EXTI->RTSR &= ~EXTI_RTSR_TR17;  // Disable rising edge trigger for line 17
	RTC->WPR = 0xFF;
	PWR->CR &= ~PWR_CR_DBP;
	NVIC_SetPriority(RTC_Alarm_IRQn, 0);
	NVIC_EnableIRQ(RTC_Alarm_IRQn);
}
void RTC_Alarm_IRQHandler(void) {
	 if (RTC->ISR & RTC_ISR_ALRAF_Msk) {
		 RTC->ISR &= ~RTC_ISR_ALRAF_Msk;    // Clear alarm flag
		 EXTI->PR = EXTI_PR_PR17;
		 GPIOA->ODR^=(1<<(5));
	 }
}







