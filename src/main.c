#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"


void GPIO_init();
void TIM1_init(uint16_t Period, uint16_t Prescaler);
void DMA2_init();
void DMA2_start(uint32_t src_addr, uint32_t dest_addr, uint32_t src_len);

uint8_t data[] = {0xFF, 0x00};

void main(void){
  
  GPIO_init();
  TIM1_init((uint16_t)499, (uint16_t)15999); // period = 0.5s
  DMA2_init();
  DMA2_start((uint32_t)data, (uint32_t)&GPIOA->ODR, 2);

  // Update DMA request enable.
  SET_BIT(TIM1->DIER, TIM_DIER_UDE);

  while (1);
}