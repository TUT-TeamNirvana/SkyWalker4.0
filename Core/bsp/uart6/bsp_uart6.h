#ifndef __BSP_UART6_H
#define __BSP_UART6_H

#include "usart.h"
#include "stdint.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"

#define UART6_DMA_RX_SIZE 256      // DMA接收缓存大小

void BSP_UART6_Init(void);

void UART6_SendData(uint8_t *data, uint16_t len);
void UART6_Print(const char *fmt, ...);

void UART6_RegisterLineCallback(void (*cb)(char *str));

#endif
