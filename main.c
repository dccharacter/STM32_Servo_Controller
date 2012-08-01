#include "stm32f10x_tim.h"
#include "./hrdw_cfg/hrdw_cfg.h"
#include <stdio.h>

int main(void)
{
	Hardware_Configuration();
	printf("\fHi!%c\r\n", ' ');
	uint16_t tim2, tim3, tim4;
	uint16_t k = 0;

    while(1)
    {
    	tim2 = TIM_GetCounter(TIM2);
    	tim3 = TIM_GetCounter(TIM3);
    	tim4 = TIM_GetCounter(TIM4);
    	printf("tim2 = %d, tim3 = %d, tim4 = %d;\r\n", tim2, tim3, tim4);
    	while (++k);
    }
}
