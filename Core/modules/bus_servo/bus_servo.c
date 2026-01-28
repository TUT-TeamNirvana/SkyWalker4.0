#include "bus_servo.h"
#include <string.h>

// 你已有的 UART8 发送函数
extern void UART8_SendData(uint8_t *data, uint16_t len);

#define CMD_SERVO_MOVE 0x03

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void BusServo_SendFrame(uint8_t cmd, const uint8_t *prm, uint8_t n)
{
    uint8_t buf[64];
    uint8_t len = (uint8_t)(n + 2);   // Length = N + 2  :contentReference[oaicite:1]{index=1}

    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = len;
    buf[3] = cmd;

    if (n && prm) memcpy(&buf[4], prm, n);

    UART8_SendData(buf, (uint16_t)(4 + n));  // 纯二进制发送
}

// 只控制 ID=1、2 两个舵机
void BusServo_Move2(uint16_t time_ms, uint16_t pos1, uint16_t pos2)
{
    uint8_t prm[9];
    uint8_t i = 0;

    prm[i++] = 2; // count=2
    prm[i++] = (uint8_t)(time_ms & 0xFF);        // time_L
    prm[i++] = (uint8_t)((time_ms >> 8) & 0xFF); // time_H

    prm[i++] = 1;                                // id1
    prm[i++] = (uint8_t)(pos1 & 0xFF);           // pos1_L
    prm[i++] = (uint8_t)((pos1 >> 8) & 0xFF);    // pos1_H

    prm[i++] = 2;                                // id2
    prm[i++] = (uint8_t)(pos2 & 0xFF);           // pos2_L
    prm[i++] = (uint8_t)((pos2 >> 8) & 0xFF);    // pos2_H

    BusServo_SendFrame(0x03, prm, i); // i=9 => Length=0x0B
}

// 角度(deg)→位置值(pos) 的通用线性映射（你按舵机实际范围配置）
uint16_t BusServo_AngleToPos(float deg,
                             float deg_min, float deg_max,
                             uint16_t pos_min, uint16_t pos_max)
{
    if (deg_max <= deg_min) return pos_min;

    deg = clamp_f(deg, deg_min, deg_max);
    float t = (deg - deg_min) / (deg_max - deg_min);
    float p = (float)pos_min + t * (float)(pos_max - pos_min);

    // 四舍五入
    int32_t pi = (int32_t)(p + 0.5f);
    if (pi < 0) pi = 0;
    return clamp_u16((uint16_t)pi,
                     (pos_min < pos_max ? pos_min : pos_max),
                     (pos_min < pos_max ? pos_max : pos_min));
}
