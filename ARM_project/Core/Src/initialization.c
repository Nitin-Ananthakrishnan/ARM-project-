#include "stm32f4xx.h"
//#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "initialization.h"  // Include the header for the declaration
#include "LCD.h"
#include "USART.h"

volatile bool display_on = false;
volatile bool DRIP_TOGGLE_PIN = false;
volatile bool FAN_TOGGLE_PIN = false;

volatile uint8_t last_exti0_state = 0;
volatile uint8_t last_exti1_state = 0;
volatile uint8_t last_exti2_state = 0;
volatile uint8_t last_exti3_state = 0;

#define DEBOUNCE_DELAY_MS 200
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
	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1|GPIO_MODER_MODE7|GPIO_MODER_MODE8);  // Input mode for PA0 and PA1

	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;  // Pull-down for PA0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1;  // Pull-down for PA1
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_1;  // Pull-down for PA2
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_1;  // Pull-down for PA3

	 GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE6); // Set PA4 and PA6 as input mode
	    // Enable pull-down resistors for PA4 and PA6 (you can also use pull-up if needed)
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR6_1);
	// Configure PA2, PA3, PA4, PA5 as input for push-button switches
//	GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5); // Input mode
//	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1;  // Pull-down for PA4
//	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_1;  // Pull-down for PA5
//
//	SYSCFG->EXTICR[0] &= ~((0xF << 8)|(0xF << 4)|(0xF << 0)|(0xF<<12));
//	SYSCFG->EXTICR[0] |=  (0x0 << 8)|(0x0 << 4)|(0x0 << 0)|(0x0<<12);

	SYSCFG->EXTICR[0] &= ~(
	    (0xF << 0)  |  // Clear EXTI0 bits
	    (0xF << 4)   // Clear EXTI1 bits
	);
	SYSCFG->EXTICR[1] &= ~((0xF << 4) | (0xF << 8));
	SYSCFG->EXTICR[1] |= ((0x0 << 4) | (0x0 << 8));
	// Set EXTI lines 0 to 3 to 0 for GPIOA
	SYSCFG->EXTICR[0] |= (
	    (0x0 << 0)  |  // EXTI0 -> PA0
	    (0x0 << 4)  |  // EXTI1 -> PA1
	    (0x0 << 8)  |  // EXTI2 -> PA2
	    (0x0 << 12)    // EXTI3 -> PA3
	);
	//Intrupt configuration starts here
	// Configure EXTI0 for PA0
	EXTI->IMR |= (1<<0)|(1<<1)|(1<<7)|(1<<8);
	EXTI->FTSR |= (1<<0)|(1<<1)|(1<<7)|(1<<8);
	EXTI->RTSR &= ~((1<<0)|(1<<1)|(1<<7)|(1<<8));
	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);  // Enable EXTI0 interrupt for PA0
	NVIC_SetPriority(EXTI1_IRQn, 1);
	NVIC_EnableIRQ(EXTI1_IRQn);  // Enable EXTI1 interrupt for PA1
	NVIC_SetPriority(EXTI9_5_IRQn, 1);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}
void EXTI0_IRQHandler(void) {
    // Check if interrupt occurred on EXTI0 (PA0)
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;  // Clear the interrupt pending bit for EXTI0
        // Toggle action for PA0 (e.g., LED or a variable)
        uint8_t state1 = (GPIOA->IDR & (1<<0)) != 0;
		if (state1 && !last_exti1_state) {
			lcd_cmd(0x01);
			lcd_set_cursor(0,0);
			FAN_TOGGLE_PIN^=1;
			if(FAN_TOGGLE_PIN==1){
						lcd_print("Fan -> Auto");}
						else{lcd_print("Fan -> Manual");}
			  // Assume you have a function to toggle an LED or change state
			timer(200);
			menu_show_main();
			GPIOA->ODR^=(1<<(5));
		}
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        uint8_t state = (GPIOA->IDR & (1<<1)) != 0;
        if (state && !last_exti1_state) {
        	lcd_cmd(0x01);
			lcd_set_cursor(0,0);
			DRIP_TOGGLE_PIN^= 1;
			if(DRIP_TOGGLE_PIN==1){
			lcd_print("Drip -> Auto");}
			else{lcd_print("Drip -> Manual");}
			timer(1000);
			menu_show_main();
            GPIOA->ODR ^= (1 << 5);
        }
        last_exti1_state = state;
    }
}
void EXTI9_5_IRQHandler(void){
    if (EXTI->PR & (1 << 7)) {
        EXTI->PR = (1 << 7); // Clear pending
        // Handle PA7 interrupt
        uint8_t state = (GPIOA->IDR & (1<<2)) != 0;
                if (state && !last_exti2_state) {
                    NVIC_SystemReset();
                    timer(100);
                }
                last_exti2_state = state;
    }
    if (EXTI->PR & (1 << 8)) {
        EXTI->PR = (1 << 8); // Clear pending
        // Handle PA8 interrupt
        uint8_t state = (GPIOA->IDR & (1<<3)) != 0;
               if (state && !last_exti3_state) {
                   display_on ^= 1;
                   timer(1000);
                   GPIOA->ODR ^= (1 << 5);
               }
               last_exti3_state = state;
    }
}


void timer_init(){
	 RCC->AHB1ENR|=0X00000001;
	 GPIOA->MODER|=(1<<10);
	 GPIOA->OTYPER|=0X00000000;
	 GPIOA->OSPEEDR|=0X00000000;
	 GPIOA->PUPDR|=0X00000000;
}
void serial_init(){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	    // Configure PB1, PB2, PB3 as general purpose output
	    GPIOB->MODER &= ~((3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2))); // Clear
	    GPIOB->MODER |=  ((1 << (1 * 2)) | (1 << (2 * 2)) | (1 << (3 * 2))); // Set as output (01)

	    // Push-pull
	    GPIOB->OTYPER &= ~((1 << 1) | (1 << 2) | (1 << 3));

	    // Medium speed
	    GPIOB->OSPEEDR |= ((1 << (1 * 2)) | (1 << (2 * 2)) | (1 << (3 * 2)));

	    // No pull-up/pull-down
	    GPIOB->PUPDR &= ~((3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2)));

}
void demux_select(uint8_t value)
{
    value &= 0x07;

    uint8_t inv = (~value) & 0x07;   // INVERT for your wiring

    if (inv & 0x01) GPIOB->BSRR = (1 << 1);
    else            GPIOB->BSRR = (1 << (1 + 16));

    if (inv & 0x02) GPIOB->BSRR = (1 << 2);
    else            GPIOB->BSRR = (1 << (2 + 16));

    if (inv & 0x04) GPIOB->BSRR = (1 << 3);
    else            GPIOB->BSRR = (1 << (3 + 16));
}

void run_progress_timer(uint8_t mode)
{
    uint32_t interval;
    uint32_t total_time;

    if (mode == 1)
    {
        interval = 120000;    // 2 minutes
        total_time = 600000;  // 10 minutes
    }
    else
    {
        interval = 180000;    // 3 minutes
        total_time = 900000;  // 15 minutes
    }

    uint32_t elapsed_total = 0;

    for (uint8_t step = 0; step < 5; step++)
    {
        uint32_t elapsed_step = 0;

        while (elapsed_step < interval)
        {
            // Stop immediately if time finished
            if (elapsed_total >= total_time)
                goto END_TIMER;

            demux_select(step);   // LED ON
            timer(300);
            elapsed_step += 300;
            elapsed_total += 300;

            if (elapsed_total >= total_time)
                goto END_TIMER;

            demux_select(7);      // LED OFF
            timer(300);
            elapsed_step += 300;
            elapsed_total += 300;
        }
    }

END_TIMER:
    demux_select(7);  // turn all LEDs OFF and exit function
    return;
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
	EXTI->FTSR &= ~EXTI_FTSR_TR17;  // Rising edge trigger
	EXTI->RTSR |= EXTI_RTSR_TR17;  // Disable rising edge trigger for line 17
	RTC->WPR = 0xFF;
	PWR->CR &= ~PWR_CR_DBP;
	NVIC_SetPriority(RTC_Alarm_IRQn, 0);
	NVIC_EnableIRQ(RTC_Alarm_IRQn);
}
void RTC_Alarm_IRQHandler(void) {
	 if (RTC->ISR & RTC_ISR_ALRAF_Msk) {
		 RTC->ISR &= ~RTC_ISR_ALRAF;    // Clear alarm flag
		 EXTI->PR = EXTI_PR_PR17;
		 if (DRIP_TOGGLE_PIN) {
			 GPIOA->ODR^=(1<<(5));
			 if(drip_time==10){
				 //run_progress_timer(1);
			 }
			 if(drip_time==15){
				 //run_progress_timer(0);
			 }
		 	 }
	 }
}








