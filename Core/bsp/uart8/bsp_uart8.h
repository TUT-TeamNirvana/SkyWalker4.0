#pragma once

#include "main.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// 你可以按需要改大一点
#ifndef UART8_DMA_RX_SIZE
#define UART8_DMA_RX_SIZE 256
#endif

void BSP_UART8_Init(void);

void UART8_SendData(uint8_t *data, uint16_t len);
void UART8_Print(const char *fmt, ...);

void UART8_RegisterLineCallback(void (*cb)(char *str));
