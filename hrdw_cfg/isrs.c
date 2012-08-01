#include "./hrdw_cfg/hrdw_cfg.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"

void USART3_IRQHandler(void)
{
	do
	{
		/* Fill the buffer RxBuffer2_SW used for final data store  */
		if (RxBufferCirc[RxCounterCirc]=='\r' || RxBufferCirc[RxCounterCirc]=='\n')
			serialDataReady = 1;

		RxBuffer_SW[RxCounter_SW++] = RxBufferCirc[RxCounterCirc++];

	}  while((RxCounterCirc < (FIFO_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3))));


	if(RxCounterCirc == FIFO_SIZE)
	{
		RxCounterCirc = 0;
	}

}

void DMA1_Channel3_IRQHandler (void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC3);
	GPIO_SetBits(GPIOC, GPIO_Pin_8);
	if (!strcmp((const char*)RxBufferCirc,"rstrst"))
	{
		while(1); /* wait until reset */
	}

}
