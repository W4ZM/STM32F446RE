#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"

void usart_init(void);
void dma_it_init();
void dma_it_start(uint32_t src_addr, uint32_t src_len);
void dma1_stream6_handler();

char msg[] = "This message is from DMA !\r\n";

void main(void){
  
  usart_init();

  dma_it_init();
  dma_it_start((uint32_t)msg, strlen(msg));

  // a small delay to be able to see the UART message.
  for (uint32_t i = 0; i < 500000; i++);

  //Enable UART in DMA mode
  SET_BIT(USART2->CR3, USART_CR3_DMAT);

  while (1);
}

void dma1_stream6_handler(){

  //Disable UART in DMA mode
  CLEAR_BIT(USART2->CR3, USART_CR3_DMAT);

  // transfer error interrupt.
  if (READ_BIT(DMA1->HISR, DMA_HISR_TEIF6) != RESET)
  {
    // disable the interrupt.
    CLEAR_BIT(DMA1_Stream6->CR, DMA_SxCR_TEIE);

    // clear the interrupt flag.
    SET_BIT(DMA1->HIFCR, DMA_HIFCR_CTEIF6);

    while(1);
  }

  // direct mode error interrupt.
  if (READ_BIT(DMA1->HISR, DMA_HISR_DMEIF6) != RESET)
  {
    // clear the interrupt flag.
    SET_BIT(DMA1->HIFCR, DMA_HIFCR_CDMEIF6);
    while(1);
  }
  
  // transfer complete interrupt.
  if (READ_BIT(DMA1->HISR, DMA_HISR_TCIF6) != RESET)
  { 
    // disable the interrupt.
    CLEAR_BIT(DMA1_Stream6->CR, DMA_SxCR_TCIE);
    
    // clear the interrupt flag.
    SET_BIT(DMA1->HIFCR, DMA_HIFCR_CTCIF6);
    return;
  }
}