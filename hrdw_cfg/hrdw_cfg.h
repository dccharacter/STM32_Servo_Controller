#ifndef HRDW_CFG_H
#define HRDW_CFG_H

#include "stm32f10x.h"

#define BufferSize             250
#define FIFO_SIZE              200

uint8_t RxBufferCirc[FIFO_SIZE];
extern uint8_t RxBuffer_SW[BufferSize];
extern uint8_t RxCounter_SW_UP;
extern uint8_t RxCounter_SW_DOWN;
uint8_t RxCounterCirc;

extern uint16_t ADC_results[2];

extern uint8_t serialDataReady;

void Hardware_Configuration (void);

#endif
