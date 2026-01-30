#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdint.h>
#include <string.h>

char printf_buf[256]={0};//用于rtt终端printf输出时进行字符串转换的缓存区
#ifdef CCMRAM
__attribute__((section(".ccmram"))) uint8_t rtt_buffer[CCMRAM_BUFFER_SIZE_UP];
#endif

void BSPLogInit()
{
    SEGGER_RTT_Init();
    #ifdef CCMRAM
    SEGGER_RTT_ConfigUpBuffer(BUFFER_INDEX, "rtt_buffer", rtt_buffer, CCMRAM_BUFFER_SIZE_UP, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    #endif
}
//不能打印浮点类型的数据
int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}
//通过这个函数将浮点类型的数据转换为字符串类型
void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}

//rtt波形显示函数
/**
  * @brief          RTT波形打印，格式为vofa的justfloat协议，参数为float类型
  * @note           该函数通过变参的形式传入多个float参数，并将其作为波形数据发送到RTT通道(注意不能直接传入指针，只能传入值)。
  * @param[in]      num_args : 参数数目
  * @retval         none
  */
  typedef float float32_t;
void RTT_PrintWave_vofa(int num_args, ...) {
    float send_cache[num_args]; // 缓存参数
    va_list arg;
    va_start(arg, num_args);
    for (int i = 0; i < num_args; i++) {
        send_cache[i] = va_arg(arg, double);
    }
    va_end(arg);
    // 2. 直接发送数据块：float数据(4*num) + 帧尾(4字节)
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    
    // 直接将 float 数组作为字节流写入 RTT Channel 1
    // 只有两次必要的写入：数据本身和帧尾
    SEGGER_RTT_Write(BUFFER_INDEX, send_cache, num_args * sizeof(float));
    SEGGER_RTT_Write(BUFFER_INDEX, tail, 4);
}
