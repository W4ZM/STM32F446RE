#include <stdint.h>
#include "stm32f4xx.h"


void usart_init(void){
    
    // On reset the 16 MHz internal RC oscillator is selected as the default CPU clock.
    // (APB1 and AHB) prescaler = 1.
    CLEAR_BIT(RCC->CFGR, (RCC_CFGR_PPRE1_2 | RCC_CFGR_HPRE_3));

    // AHB clock enable.
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    // Delay after an RCC peripheral clock enabling
    volatile uint32_t tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    (void)tmpreg;

    // APB1 clock enable.
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
    // Delay after an RCC peripheral clock enabling
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
    (void)tmpreg;



    // Configure the USART2 peripheral.

    // with no prescaling, ABP1 is 16MHZ
    // we choose BaudRate = 38400 Bps.
    // DIV_Mantissa = 26 and DIV_Fraction = 1
    MODIFY_REG(USART2->BRR,
        (USART_BRR_DIV_Mantissa_Msk | USART_BRR_DIV_Fraction_Msk),
        ((26 << USART_BRR_DIV_Mantissa_Pos) | (1 << USART_BRR_DIV_Fraction_Pos))
    );

    // 16x oversampling
    CLEAR_BIT(USART2->CR1, USART_CR1_OVER8);

    // 8-bit USART Word Length.
    CLEAR_BIT(USART2->CR1, USART_CR1_M);

    // no parity check and just one stop bit.
    CLEAR_BIT(USART2->CR1, USART_CR1_PCE);
    CLEAR_BIT(USART2->CR2, (USART_CR2_STOP_0 | USART_CR2_STOP_1));

    // we disable any form of Hardware Flow Control.
    CLEAR_BIT(USART2->CR3, USART_CR3_CTSE);
    CLEAR_BIT(USART2->CR3, USART_CR3_RTSE);

    // configure USART mode to transmit only.
    SET_BIT(USART2->CR1, USART_CR1_TE);

    // Enable USART.
    SET_BIT(USART2->CR1, USART_CR1_UE);



    // GPIOA Configuration
    // PA2 ------> USART2_TX
    
    // push-pull output type.
    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT2);

    // alternate function mode.
    CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE2_0);
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODE2_1);

    // No Pull-up or Pull-down activation
    CLEAR_BIT(GPIOA->PUPDR, (GPIO_PUPDR_PUPD2_0 | GPIO_PUPDR_PUPD2_1));

    // low output speed.
    CLEAR_BIT(GPIOA->OSPEEDR, (GPIO_OSPEEDR_OSPEED2_0 | GPIO_OSPEEDR_OSPEED2_1));

    // this depends on the specific STM32 MCU for me it's AF7.
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFRL2, (0x07U << GPIO_AFRL_AFSEL2_Pos));
}

void usart_wait_for_flag(uint32_t mask, bool flag)
{
    for (;;)
    {
        if (((USART2->SR & mask) == mask) && (SET == flag)) return;
        if (((USART2->SR & mask) != mask) && (RESET == flag)) return;
    }
}

void usart_send(const uint8_t* data, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        // wait for TXE flag to be set in order to write data in DR.
        usart_wait_for_flag(USART_SR_TXE, SET);

        // write data in DR (8 bits).
        WRITE_REG(USART2->DR, (uint8_t)(data[i] & (uint8_t)0xFF));
    }

    // wait for TC flag.
    usart_wait_for_flag(USART_SR_TC, SET);
}