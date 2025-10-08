#include <stdint.h>
#include "stm32f4xx.h"

volatile uint32_t tmpreg;

void usart_init(void){
    
    // On reset the 16 MHz internal RC oscillator is selected as the default CPU clock.
    // (APB1 and AHB) prescaler = 1.
    CLEAR_BIT(RCC->CFGR, (RCC_CFGR_PPRE1_2 | RCC_CFGR_HPRE_3));

    // AHB clock enable.
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    // Delay after an RCC peripheral clock enabling
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
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

void dma_it_init(){

    // enable DMA1 clock.
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);
    (void)tmpreg;

    // set Stream6_IRQ preempt priority and sub-priority to the highest and enable it's interrupt.
    NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    // It is forbidden to write these registers when the EN bit is read as 1.
    CLEAR_BIT(DMA1_Stream6->CR, DMA_SxCR_EN);

    // select channel 4.
    MODIFY_REG(DMA1_Stream6->CR, DMA_SxCR_CHSEL, 4 << DMA_SxCR_CHSEL_Pos);

    // use memory-to-peripheral transfer direction.
    MODIFY_REG(DMA1_Stream6->CR, DMA_SxCR_DIR, 1 << DMA_SxCR_DIR_Pos);

    // enable memory increment mode and disable peripheral one.
    MODIFY_REG(DMA1_Stream6->CR, (DMA_SxCR_MINC | DMA_SxCR_PINC), 1 << DMA_SxCR_MINC_Pos);

    // 1 byte peripheral data size (PSIZE).
    // (In direct mode, MSIZE is forced by hardware to the same value as PSIZE as soon as EN = 1).
    MODIFY_REG(DMA1_Stream6->CR, DMA_SxCR_PSIZE, 0);

    // DMA is the flow controller.
    // Circular mode disabled.
    // No buffer switching at the end of transfer.
    MODIFY_REG(DMA1_Stream6->CR, (DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_PFCTRL), 0);

    // set the software priority level of the stream to low.
    MODIFY_REG(DMA1_Stream6->CR, DMA_SxCR_PL, 0);

    // disable FIFO mode.
    CLEAR_BIT(DMA1_Stream6->FCR, DMA_SxFCR_DMDIS);
}

void dma_it_start(uint32_t src_addr, uint32_t src_len){

    // provide dma stream data lenght to NDTR.
    WRITE_REG(DMA1_Stream6->NDTR, src_len);

    // set dma stream destination address to usart2 data register address.
    WRITE_REG(DMA1_Stream6->PAR, (uint32_t)&USART2->DR);

    // set dma stream source address to our memory buffer address.
    WRITE_REG(DMA1_Stream6->M0AR, src_addr);

    // clear interrupt flags for TC, TE, DME.
    WRITE_REG(DMA1->HIFCR, DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6);

    // enable TC, TE, DME interrupts.
    MODIFY_REG(DMA1_Stream6->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE,
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE
    );
    
    // enable the stream.
    SET_BIT(DMA1_Stream6->CR, DMA_SxCR_EN);
}   