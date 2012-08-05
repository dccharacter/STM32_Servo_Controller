#include "./hrdw_cfg/hrdw_cfg.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "./clock/clock.h"

void USART3_IRQHandler(void)
{
	do
	{
		/* Fill the buffer RxBuffer2_SW used for final data store  */
		//if (RxBufferCirc[RxCounterCirc]=='\r' || RxBufferCirc[RxCounterCirc]=='\n')
		//	serialDataReady = 1;

		RxBuffer_SW[RxCounter_SW_UP++] = RxBufferCirc[RxCounterCirc++];

	}  while((RxCounterCirc < (FIFO_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3))));


	if(RxCounterCirc == FIFO_SIZE)
	{
		RxCounterCirc = 0;
	}

	if(RxCounter_SW_UP == BufferSize)
	{
		RxCounter_SW_UP = 0;
	}

}

void DMA1_Channel3_IRQHandler (void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC3);
}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Toggle LED1 */

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		/* Toggle LED1 */

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		/* Toggle LED1 */

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		/* Toggle LED1 */

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		/* Toggle LED1 */

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line10);
	}

	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		/* Toggle LED1 */

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line11);
	}

	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		/* Toggle LED1 */

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}

void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}
