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
static void PosPID_Init(PosPID_t *pid,
                        float kp, float ki, float kd,
                        float max_rpm)
{
    pid->max_rpm = (max_rpm < 0.0f) ? -max_rpm : max_rpm;  // 输出限幅
    pid->ref_ticks = 0;  // 初始化目标位置
    PID_Init(&pid->pid, kp, ki, kd, pid->max_rpm);
}
// 设置位置环PID时间间隔
static void PosPID_SetDt(PosPID_t *pid, float dt_s){
    PID_SetDt(&pid->pid, dt_s);
}
// 设置目标位置
static void PosPID_SetRef(PosPID_t *pid, int32_t ref_ticks){
    pid->ref_ticks = ref_ticks;
}
// 计算位置环
static float PosPID_Calc(PosPID_t *pid, int32_t fb_ticks)
{
    // 相同的调用
    float out = PID_Calc(&pid->pid, (float)pid->ref_ticks, (float)fb_ticks);
    // 双重保险限幅
    if (out > pid->max_rpm) out = pid->max_rpm;
    else if (out < -pid->max_rpm) out = -pid->max_rpm;
    return out;
}

#endif //POS_PID_H
