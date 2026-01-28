#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    // 传入3个参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_max = max_output;
    // 累加积分清零
    pid->integral = 0;
    // 上一次的误差清零
    pid->last_error = 0;
    // 输出清零
    pid->output = 0;
}

float PID_Calc(PID_t *pid, float ref, float feedback)
{
    // 计算一手预期和反馈的误差
    float error = ref - feedback;
    // 把当前误差累加到积分项里
    pid->integral += error;
    float derivative = error - pid->last_error;
    pid->last_error = error;

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}
