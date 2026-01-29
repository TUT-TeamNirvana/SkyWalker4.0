//
// Created by 24699 on 26-1-29.
//

#ifndef POS_PID_H
#define POS_PID_H


#include "pid.h"
#include <stdint.h>

typedef struct
{
    PID_t pid;          // 位置环PID：输入 ticks，输出 rpm（速度给定）
    float max_rpm;      // 输出限幅（rpm）
    int32_t ref_ticks;  // 目标位置（ticks）
} PosPID_t;

/**
 * @brief 初始化位置环（ticks -> rpm）
 * @param kp ki kd      位置环PID参数（输出单位：rpm）
 * @param max_rpm       输出速度限幅（rpm）
 */
static inline void PosPID_Init(PosPID_t *p,
                                     float kp, float ki, float kd,
                                     float max_rpm)
{
    p->max_rpm = (max_rpm < 0.0f) ? -max_rpm : max_rpm;
    p->ref_ticks = 0;
    PID_Init(&p->pid, kp, ki, kd, p->max_rpm);
}

/**
 * @brief 设置 dt（如果你的 PID 已加 dt，就保留；否则你可以删掉这个函数）
 */
static inline void PosPID_SetDt(PosPID_t *p, float dt_s)
{
    PID_SetDt(&p->pid, dt_s);
}

/** 设置目标位置（ticks，多圈连续） */
static inline void PosPID_SetRef(PosPID_t *p, int32_t ref_ticks)
{
    p->ref_ticks = ref_ticks;
}

/**
 * @brief 计算位置环输出（速度给定 rpm）
 * @param fb_ticks 当前反馈位置（ticks，多圈连续）
 */
static inline float PosPID_Calc(PosPID_t *p, int32_t fb_ticks)
{
    float out = PID_Calc(&p->pid, (float)p->ref_ticks, (float)fb_ticks);

    // 双重保险限幅
    if (out > p->max_rpm) out = p->max_rpm;
    else if (out < -p->max_rpm) out = -p->max_rpm;

    return out;
}

#endif //POS_PID_H
