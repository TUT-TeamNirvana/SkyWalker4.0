#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    // 传入3个参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_max = max_output;
    pid->dt = 1.0f;
    // 累加积分清零
    pid->integral = 0;
    // 默认关闭积分限幅（需要时用 PID_SetIntegralLimit 打开）
    pid->integral_max = 0.0f;
    // 上一次的误差清零
    pid->last_error = 0;
    // 输出清零
    pid->output = 0;
}

void PID_SetDt(PID_t *pid, float dt_s){
    if (dt_s > 0.0f)  pid->dt = dt_s;
}
// 积分限幅设置
void PID_SetIntegralLimit(PID_t *pid, float i_max){
    if (i_max < 0.0f) i_max = -i_max;  // 负数容错
    pid->integral_max = i_max; // i_max=0 表示关闭（PID_Calc里会进行判断）
}

float PID_Calc(PID_t *pid, float ref, float feedback)
{
    // 计算一手预期和反馈的误差
    float error = ref - feedback;
    // 把当前误差累加到积分项里
    // 积分项
    pid->integral += error * pid->dt;
    // 积分限幅（可选）
    if (pid->integral_max > 0.0f){  // 限制一手最大限度
        if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
        else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    }
    // 微分项
    float derivative = (error - pid->last_error) / pid->dt;
    pid->last_error = error;  // 更新误差

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 输出限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}
