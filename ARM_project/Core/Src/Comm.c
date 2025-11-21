#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "Comm.h"

#define GPIO_PIN_1      (0)  // Pin 0 (on GPIOB)
#define GPIO_PIN_2      (1)  // Pin 1 (on GPIOB)
#define READ_SIGNAL_PIN (3)

#define DRIP10_STATE    0  // 00
#define DRIP15_STATE    1  // 01
#define FAN_ON_STATE    2  // 10
#define FAN_OFF_STATE   3  // 11

void GPIO_comm_Init(void)
{
    // Enable the GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure pins as output (mode 01 for general-purpose output)
    GPIOB->MODER &= ~(0x3 << (2 * GPIO_PIN_1));  // Clear mode for pin 0
    GPIOB->MODER |= (0x1 << (2 * GPIO_PIN_1));   // Set as output for pin 0

    GPIOB->MODER &= ~(0x3 << (2 * GPIO_PIN_2));  // Clear mode for pin 1
    GPIOB->MODER |= (0x1 << (2 * GPIO_PIN_2));   // Set as output for pin 1

    // Configure Read_Signal pin as input
    GPIOB->MODER &= ~(0x3 << (2 * READ_SIGNAL_PIN));  // Set pin as input (00)
}

// Set the state of the system (two-bit encoding)
void SetState(uint8_t state)
{
    switch (state)
    {
        case DRIP10_STATE:
            GPIOB->ODR &= ~(1 << GPIO_PIN_1);  // Pin 1 = 0
            GPIOB->ODR &= ~(1 << GPIO_PIN_2);  // Pin 2 = 0
            break;

        case DRIP15_STATE:
            GPIOB->ODR &= ~(1 << GPIO_PIN_1);  // Pin 1 = 0
            GPIOB->ODR |= (1 << GPIO_PIN_2);   // Pin 2 = 1
            break;

        case FAN_ON_STATE:
            GPIOB->ODR |= (1 << GPIO_PIN_1);   // Pin 1 = 1
            GPIOB->ODR &= ~(1 << GPIO_PIN_2);  // Pin 2 = 0
            break;

        case FAN_OFF_STATE:
            GPIOB->ODR |= (1 << GPIO_PIN_1);   // Pin 1 = 1
            GPIOB->ODR |= (1 << GPIO_PIN_2);   // Pin 2 = 1
            break;

        default:
            break;
    }
}
uint8_t ReadSignal(void)
{
    // Read the GPIOB input data register (IDR) for the Read_Signal pin
    uint8_t signal = (GPIOB->IDR & (1 << READ_SIGNAL_PIN)) >> READ_SIGNAL_PIN;
    return signal;  // Returns 0 (LOW) or 1 (HIGH)
}
