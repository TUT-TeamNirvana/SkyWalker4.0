#include "pid.h"

// 用于常做的范围限制
static float clampf(float x, float min_v, float max_v){
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return x;
}

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
    // 默认关闭抗积分饱和
    pid->anti_windup_en = 0;
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
// 抗积分饱和设置
void PID_EnableAntiWindup(PID_t *pid, uint8_t enable){
    pid->anti_windup_en = (enable != 0);  // 0关1开
}

float PID_Calc(PID_t *pid, float ref, float feedback)
{
    // 计算一手预期和反馈的误差
    float error = ref - feedback;
    float P = pid->Kp * error;  // 先计算P
    float derivative = (error - pid->last_error) / pid->dt;  // 微分项
    float D = pid->Kd * derivative;  // 再计算D
    pid->last_error = error;  // 更新误差
    // 把当前误差累加到积分项里
    // 积分项 （先生成一手候选积分方便后期做抗积分饱和，不做可以直接当作积分项）
    float integral_candidate = pid->integral + error * pid->dt;
    // 积分限幅（可选）
    if (pid->integral_max > 0.0f){
        integral_candidate = clampf(integral_candidate, -pid->integral_max, pid->integral_max);
    }
    // 先用预选积分算一次PID看结果
    float u_candidate = P + pid->Ki * integral_candidate + D;
    float u_sat = clampf(u_candidate, -pid->output_max, pid->output_max);  // 物理输出限幅
    // 抗积分饱和
    if (pid->anti_windup_en){  // 看看有没有启动抗饱和
        // 若已饱和，且误差方向会让输出“更饱和”，则拒绝本次积分更新
        // u_candidate > +max 且 error>0：继续积分只会更大（加深正饱和）
        // u_candidate < -max 且 error<0：继续积分只会更小（加深负饱和）
        // 如果上面两种状态都没有触发，证明没毛病，继续积分
        uint8_t deepen_sat =
            ((u_candidate >  pid->output_max) && (error > 0.0f)) ||
            ((u_candidate < -pid->output_max) && (error < 0.0f));
        if (!deepen_sat){
            pid->integral = integral_candidate;  // 允许积分
        }
        // 否则直接不更新 冻结一手integral
    }
    else{  // 不开抗饱和 直接采用候选积分
        pid->integral = integral_candidate;
    }

    // 最终计算
    pid->output = P + pid->Ki * pid->integral + D;
    pid->output = clampf(pid->output, -pid->output_max, pid->output_max);  // 输出限幅

    return pid->output;
}
