#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <math.h>

// PID结构体
typedef struct
{
    float Kp;  // 比例系数 P
    float Ki;  // 积分系数 I
    float Kd;  // 微分系数 D
    float dt;

    float integral;  // 积分累加项（把历史误差累加起来）
    float integral_max;  // 一手积分限幅（限制积累的积分最大值）
    uint8_t anti_windup_en; // 抗积分饱和开关（0=关，1=开）

    float last_error;  // 上一次的误差（用于计算微分项）
    float output;  // 本次计算得到的输出（也保存下来）
    float output_max;  // 输出限幅的最大绝对值（最后会把输出夹在 [-output_max, +output_max]）

    // 用于速度环 feedback的一阶低通滤波
    uint8_t lpf_en;       // 0=关 1=开
    float   lpf_alpha;    // y = alpha*y_prev + (1-alpha)*x, alpha∈[0,1)
    float   lpf_state;    // 上一次 y（滤波输出）
    uint8_t lpf_inited;   // 避免启动时跳变：第一次直接置为输入
} PID_t;

// 对传入的PID结构体初始化一套PID系数以及限制最大输出
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output);
// 设置时间间隔
void  PID_SetDt(PID_t *pid, float dt_s);
// 设置一手积分限幅 （这里可以控制开关 i_max 为0的时候为关闭积分限幅）
void PID_SetIntegralLimit(PID_t *pid, float i_max);
// 抗积分饱和
void PID_EnableAntiWindup(PID_t *pid, uint8_t enable);
// PID计算函数；分别传入期望值、反馈值
float PID_Calc(PID_t *pid, float ref, float feedback);

// 用于速度环 feedback的一阶低通滤波
void PID_EnableLPF(PID_t *pid, uint8_t enable);
// 直接设置 alpha（推荐0.7~0.98，越大越越慢，滤的越猛）
void PID_SetLPFAlpha(PID_t *pid, float alpha);
// 用截止频率 fc(Hz) 来设置 alpha（更工程化）
// alpha = exp(-2*pi*fc*dt)
void PID_SetLPFCutoffHz(PID_t *pid, float fc_hz);

#endif
