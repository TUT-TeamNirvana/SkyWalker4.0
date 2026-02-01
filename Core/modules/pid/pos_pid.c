#include "pos_pid.h"

// 位置环PID初始化
void PosPID_Init(PosPID_t *pid,
                        float kp, float ki, float kd,
                        float max_rpm)
{
    pid->max_rpm = (max_rpm < 0.0f) ? -max_rpm : max_rpm;  // 输出限幅
    pid->ref_ticks = 0;  // 初始化目标位置
    pid->limit_ticks = 0;  // 初始化最小死区
    PID_Init(&pid->pid, kp, ki, kd, pid->max_rpm);
}
// 设置位置环PID时间间隔
void PosPID_SetDt(PosPID_t *pid, float dt_s){
    PID_SetDt(&pid->pid, dt_s);
}
// 设置最小死区
void PosPID_SetLimitTicks(PosPID_t *pid, int32_t limit_ticks) {
    pid->limit_ticks = limit_ticks;
}
// 设置积分限幅
void PosPID_SetIntegralLimit(PosPID_t *pid, float i_max){
    PID_SetIntegralLimit(&pid->pid, i_max);  // 调用的是内环积分限幅
}
// 设置抗积分饱和
void PosPID_EnableAntiWindup(PosPID_t *pid, uint8_t enable){
    PID_EnableAntiWindup(&pid->pid, enable);
}
// 设置目标位置
void PosPID_SetRef(PosPID_t *pid, int32_t ref_ticks){
    pid->ref_ticks = ref_ticks;
}
// 计算位置环
float PosPID_Calc(PosPID_t *pid, int32_t fb_ticks)
{
    if (abs(fb_ticks - pid->ref_ticks) <= pid->limit_ticks) {
        return 0;  // 如果在死区内部 则无需更新了
    }
    // 相同的调用
    float out = PID_Calc(&pid->pid, (float)pid->ref_ticks, (float)fb_ticks);
    // 双重保险限幅
    if (out > pid->max_rpm) out = pid->max_rpm;
    else if (out < -pid->max_rpm) out = -pid->max_rpm;
    return out;
}