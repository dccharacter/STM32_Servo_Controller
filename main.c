#include "stm32f10x_tim.h"
#include "./hrdw_cfg/hrdw_cfg.h"
#include "./gcode/gcode_parse.h"
#include <stdio.h>
#include	"./timer/timer.h"
#include	"./clock/clock.h"

uint8_t RxBuffer_SW[BufferSize];
uint8_t RxCounter_SW_UP;
uint8_t RxCounter_SW_DOWN;
uint32_t ADC_results[2];

int main(void)
{
	Hardware_Configuration();

	// say hi to host
	printf("start\n\rok\n\r");

    while(1)
    {
    	uint16_t TIMx = TIM_GetCounter(TIM4);
    	if (RxCounter_SW_DOWN != RxCounter_SW_UP)
    	{
    		gcode_parse_char(RxBuffer_SW[RxCounter_SW_DOWN]);
    		if (++RxCounter_SW_DOWN == BufferSize) RxCounter_SW_DOWN=0;
    	}

    	ifclock(clock_flag_10ms) {
    		clock_10ms();
    	}
    }
}
