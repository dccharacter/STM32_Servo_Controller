#include	"./clock/clock.h"

/** \file
	\brief Do stuff periodically
*/

#include <stdio.h>
#include "stm32f10x_gpio.h"
#include	"./pinio/pinio.h"
#include	"./serial/sersendf.h"
#include	"./dda/dda_queue.h"
//#include	"watchdog.h"
#include	"./extruder/temp.h"
#include	"./timer/timer.h"
#include	"debug.h"
#include	"./heater/heater.h"
#include	"./serial/serial.h"
#ifdef	TEMP_INTERCOM
	#include	"intercom.h"
#endif
//#include	"memory_barrier.h"

/*!	do stuff every 1/4 second

	called from clock_10ms(), do not call directly
*/
void clock_250ms() {

	#ifndef	NO_AUTO_IDLE
	if (temp_all_zero())	{
		if (psu_timeout > (30 * 4)) {
			power_off();
		}

		else {
			//uint8_t save_reg = SREG;
			//cli();
			//CLI_SEI_BUG_MEMORY_BARRIER();
			psu_timeout++;
			//MEMORY_BARRIER();
			//SREG = save_reg;
		}
	}
	#endif

	ifclock(clock_flag_1s) {
		GPIOC->ODR ^= GPIO_Pin_9;
		if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
			// current position
			update_current_position();
			printf("Pos: %d.%u,%d.%u,%d.%u,%d.%u,%lu\n", current_position.X >> 2, (uint8_t)current_position.X & 3,
					current_position.Y >> 2, (uint8_t)current_position.Y & 3,
					current_position.Z >> 2, (uint8_t)current_position.Z & 3,
					current_position.E >> 2, (uint8_t)current_position.E & 3,
					current_position.F);

			// target position
			printf("Dst: %d.%u,%d.%u,%d.%u,%d.%u,%lu\n",
					movebuffer[mb_tail].endpoint.X >> 2, movebuffer[mb_tail].endpoint.X & 3,
					movebuffer[mb_tail].endpoint.Y >> 2, movebuffer[mb_tail].endpoint.Y & 3,
					movebuffer[mb_tail].endpoint.Z >> 2, movebuffer[mb_tail].endpoint.Z & 3,
					movebuffer[mb_tail].endpoint.E >> 2, movebuffer[mb_tail].endpoint.E & 3,
					movebuffer[mb_tail].endpoint.F);

			// Queue
			print_queue();

			// newline
			serial_writechar('\n');
		}
		// temperature
		/*		if (temp_get_target())
		temp_print();*/
	}
	#ifdef	TEMP_INTERCOM
	start_send();
	#endif
}

/*! do stuff every 10 milliseconds

	call from ifclock(CLOCK_FLAG_10MS) in busy loops
*/
void clock_10ms() {
	// reset watchdog

	//wd_reset();

	temp_tick();

	ifclock(clock_flag_250ms) {
		clock_250ms();
	}

}

void TimingDelay_Decrement(void)
{

	clock_flag_10ms = 1;
	if (clock_counter_250ms++ >= 25)
	{
		clock_flag_250ms = 1;
		clock_counter_250ms = 0;
	}
	if (clock_counter_1s++ >=100)
	{
		clock_flag_1s = 1;
		clock_counter_1s = 0;
	}
}

