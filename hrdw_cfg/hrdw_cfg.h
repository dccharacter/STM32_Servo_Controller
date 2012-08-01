#ifndef HRDW_CFG_H
#define HRDW_CFG_H

#include "stm32f10x.h"

#define BufferSize             250
#define FIFO_SIZE              200

char RxBufferCirc[FIFO_SIZE];
char RxBuffer_SW[BufferSize];
char RxCounter_SW;
char RxCounterCirc;

uint8_t serialDataReady;

void Hardware_Configuration (void);

#endif
