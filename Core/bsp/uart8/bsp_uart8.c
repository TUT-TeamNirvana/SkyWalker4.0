#include "bsp_uart8.h"

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;   // ⚠️ 名字以你 CubeMX 生成的为准

static uint8_t uart8_dma_buffer[UART8_DMA_RX_SIZE];
static char    uart8_parse_buffer[UART8_DMA_RX_SIZE];

static void (*uart8_line_callback)(char *str) = NULL;

void BSP_UART8_Init(void)
{
    memset(uart8_dma_buffer, 0, UART8_DMA_RX_SIZE);

    HAL_UART_Receive_DMA(&huart8, uart8_dma_buffer, UART8_DMA_RX_SIZE);
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

    //UART8_Print("UART8 DMA Mode Started!\r\n");
}

void UART8_SendData(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart8, data, len, 100);
}

void UART8_Print(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART8_SendData((uint8_t *)buf, (uint16_t)strlen(buf));
}

void UART8_RegisterLineCallback(void (*cb)(char *str))
{
    uart8_line_callback = cb;
}

/**
 * @brief UART8 IRQ handler: IDLE triggers one-frame parsing
 */
void UART8_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
        HAL_UART_DMAStop(&huart8);

        // 计算本次收到的长度
        uint16_t received = UART8_DMA_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);

        if (received > 0 && received < UART8_DMA_RX_SIZE)
        {
            memcpy(uart8_parse_buffer, uart8_dma_buffer, received);
            uart8_parse_buffer[received] = '\0';

            if (uart8_line_callback)
                uart8_line_callback(uart8_parse_buffer);
        }

        // 重启 DMA
        memset(uart8_dma_buffer, 0, UART8_DMA_RX_SIZE);
        HAL_UART_Receive_DMA(&huart8, uart8_dma_buffer, UART8_DMA_RX_SIZE);

        // 让 HAL 有机会处理其他错误/状态
        HAL_UART_IRQHandler(&huart8);
        return;
    }

    HAL_UART_IRQHandler(&huart8);
}
