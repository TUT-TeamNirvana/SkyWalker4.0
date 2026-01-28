#include "bsp_uart6.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/* ================= DMA 接收缓冲区 ================= */
static uint8_t uart6_dma_buffer[UART6_DMA_RX_SIZE];

/* ================= 解析专用缓冲区（安全） ================= */
static char uart6_parse_buffer[UART6_DMA_RX_SIZE];

/* ================= 行接收回调 ================= */
static void (*uart6_line_callback)(char *str) = NULL;


/************************************
 * UART6 初始化：DMA + IDLE
 ************************************/
void BSP_UART6_Init(void)
{
    memset(uart6_dma_buffer, 0, UART6_DMA_RX_SIZE);
    HAL_UART_Receive_DMA(&huart6, uart6_dma_buffer, UART6_DMA_RX_SIZE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    UART6_Print("UART6 DMA Mode Started!\r\n");
}


/************************************
 * UART6 发送
 ************************************/
void UART6_SendData(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart6, data, len, 100);
}

void UART6_Print(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART6_SendData((uint8_t *)buf, strlen(buf));
}


/************************************
 * 注册回调函数
 ************************************/
void UART6_RegisterLineCallback(void (*cb)(char *str))
{
    uart6_line_callback = cb;
}


/************************************
 * USART6 中断：IDLE 触发解析
 ************************************/
void USART6_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        HAL_UART_DMAStop(&huart6);

        uint16_t received = UART6_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);

        /* ========= 长度保护 ========= */
        if (received == 0 || received >= UART6_DMA_RX_SIZE)
        {
            memset(uart6_dma_buffer, 0, UART6_DMA_RX_SIZE);
            HAL_UART_Receive_DMA(&huart6, uart6_dma_buffer, UART6_DMA_RX_SIZE);
            HAL_UART_IRQHandler(&huart6);
            return;
        }

        /* ========= 复制干净数据 ========= */
        memcpy(uart6_parse_buffer, uart6_dma_buffer, received);
        uart6_parse_buffer[received] = '\0';

        if (uart6_line_callback)
        {
            uart6_line_callback(uart6_parse_buffer);
        }

        /* ========= 清空 DMA 缓冲并重启 ========= */
        memset(uart6_dma_buffer, 0, UART6_DMA_RX_SIZE);
        HAL_UART_Receive_DMA(&huart6, uart6_dma_buffer, UART6_DMA_RX_SIZE);

        HAL_UART_IRQHandler(&huart6);
        return;
    }

    HAL_UART_IRQHandler(&huart6);
}
