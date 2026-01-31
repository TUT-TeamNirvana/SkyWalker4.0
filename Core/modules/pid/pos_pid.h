//
// Created by 24699 on 26-1-29.
//

#ifndef POS_PID_H
#define POS_PID_H

#include "pid.h"
#include <stdint.h>

// 位置环PID结构体
typedef struct
{
    PID_t pid;          // 位置环PID：输入 ticks，输出 rpm（速度给定）
    float max_rpm;      // 输出限幅（rpm）
    int32_t ref_ticks;  // 目标位置（ticks）
} PosPID_t;

// 位置环PID初始化
void PosPID_Init(PosPID_t *pid, float kp, float ki, float kd, float max_rpm);
// 设置位置环PID时间间隔
void PosPID_SetDt(PosPID_t *pid, float dt_s);
// 设置积分限幅
void PosPID_SetIntegralLimit(PosPID_t *pid, float i_max);
// 设置抗积分饱和
void PosPID_EnableAntiWindup(PosPID_t *pid, uint8_t enable);
// 设置目标位置
void PosPID_SetRef(PosPID_t *pid, int32_t ref_ticks);
// 计算位置环
float PosPID_Calc(PosPID_t *pid, int32_t fb_ticks);

#endif //POS_PID_H
