#include <stdint.h>
#include "stm32f4xx.h"

volatile uint32_t tmpreg;

void GPIO_init(){
    
    // On reset the 16 MHz internal RC oscillator is selected as the default CPU clock.
    // (APB1 and AHB) prescaler = 1.
    CLEAR_BIT(RCC->CFGR, (RCC_CFGR_PPRE2_2 | RCC_CFGR_HPRE_3));

    // AHB1 clock enable for GPIOA port.
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    // Delay after an RCC peripheral clock enabling
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
    (void)tmpreg;

    // set pins [0-7] to output mode because DMA need to write by byte.
    MODIFY_REG(GPIOA->MODER, 0x0000FFFF, 0x00005555);
}

void TIM1_init(uint16_t Period, uint16_t Prescaler){

    // APB2 clock enable for timer1.
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    // Delay after an RCC peripheral clock enabling
    tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    (void)tmpreg;

    // set the auto-reload value.
    WRITE_REG(TIM1->ARR, Period);
    
    // set prescaler value.
    WRITE_REG(TIM1->PSC, Prescaler);

    // set timer update request source to ONLY counter overflow/underflow.
    SET_BIT(TIM1->CR1, TIM_CR1_URS);

    // reinitialize the counter and generates an update of the registers.
    SET_BIT(TIM1->EGR, TIM_EGR_UG);

    // enable the counter.
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

void DMA2_init(){

    // enable DMA2 clock.
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
    (void)tmpreg;

    // set Stream5_IRQ preempt priority and sub-priority to the highest and enable it's interrupt.
    NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    // It is forbidden to write these registers when the EN bit is read as 1.
    CLEAR_BIT(DMA2_Stream5->CR, DMA_SxCR_EN);

    // select channel 6.
    MODIFY_REG(DMA2_Stream5->CR, DMA_SxCR_CHSEL, 6 << DMA_SxCR_CHSEL_Pos);

    // use memory-to-peripheral transfer direction.
    MODIFY_REG(DMA2_Stream5->CR, DMA_SxCR_DIR, 1 << DMA_SxCR_DIR_Pos);

    // enable memory increment mode and disable peripheral one.
    MODIFY_REG(DMA2_Stream5->CR, (DMA_SxCR_MINC | DMA_SxCR_PINC), 1 << DMA_SxCR_MINC_Pos);

    // 1 byte peripheral data size (PSIZE).
    // (In direct mode, MSIZE is forced by hardware to the same value as PSIZE as soon as EN = 1).
    MODIFY_REG(DMA2_Stream5->CR, DMA_SxCR_PSIZE, 0);

    // DMA is the flow controller.
    // No buffer switching at the end of transfer.
    MODIFY_REG(DMA2_Stream5->CR, (DMA_SxCR_DBM | DMA_SxCR_PFCTRL), 0);

    // enable circular mode.
    SET_BIT(DMA2_Stream5->CR, DMA_SxCR_CIRC);

    // set the software priority level of the stream to low.
    MODIFY_REG(DMA2_Stream5->CR, DMA_SxCR_PL, 0);

    // disable FIFO mode.
    CLEAR_BIT(DMA2_Stream5->FCR, DMA_SxFCR_DMDIS);
}

void DMA2_start(uint32_t src_addr, uint32_t dest_addr, uint32_t src_len){

    // provide dma stream data lenght to NDTR.
    WRITE_REG(DMA2_Stream5->NDTR, src_len);

    // set dma stream destination address.
    WRITE_REG(DMA2_Stream5->PAR, dest_addr);

    // set dma stream source address to our memory buffer address.
    WRITE_REG(DMA2_Stream5->M0AR, src_addr);

    // clear interrupt flags for TC, TE, DME.
    WRITE_REG(DMA2->HIFCR, DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5);

    // enable TC, TE, DME interrupts.
    MODIFY_REG(DMA2_Stream5->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE,
        DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE
    );
    
    // enable the stream.
    SET_BIT(DMA2_Stream5->CR, DMA_SxCR_EN);
}   