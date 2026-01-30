#include "pos_pid.h"

// 位置环PID初始化
void PosPID_Init(PosPID_t *pid,
                        float kp, float ki, float kd,
                        float max_rpm)
{
    pid->max_rpm = (max_rpm < 0.0f) ? -max_rpm : max_rpm;  // 输出限幅
    pid->ref_ticks = 0;  // 初始化目标位置
    PID_Init(&pid->pid, kp, ki, kd, pid->max_rpm);
}
// 设置位置环PID时间间隔
void PosPID_SetDt(PosPID_t *pid, float dt_s){
    PID_SetDt(&pid->pid, dt_s);
}
// 设置积分限幅
void PosPID_SetIntegralLimit(PosPID_t *pid, float i_max){
    PID_SetIntegralLimit(&pid->pid, i_max);  // 调用的是内环积分限幅
}
// 设置目标位置
void PosPID_SetRef(PosPID_t *pid, int32_t ref_ticks){
    pid->ref_ticks = ref_ticks;
}
// 计算位置环
float PosPID_Calc(PosPID_t *pid, int32_t fb_ticks)
{
    // 相同的调用
    float out = PID_Calc(&pid->pid, (float)pid->ref_ticks, (float)fb_ticks);
    // 双重保险限幅
    if (out > pid->max_rpm) out = pid->max_rpm;
    else if (out < -pid->max_rpm) out = -pid->max_rpm;
    return out;
}