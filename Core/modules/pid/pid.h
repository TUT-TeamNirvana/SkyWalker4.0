#ifndef PID_H
#define PID_H

// PID结构体
typedef struct
{
    float Kp;  // 比例系数 P
    float Ki;  // 积分系数 I
    float Kd;  // 微分系数 D
    float integral;  // 积分累加项（把历史误差累加起来）
    float last_error;  // 上一次的误差（用于计算微分项）
    float output;  // 本次计算得到的输出（也保存下来）
    float output_max;  // 输出限幅的最大绝对值（最后会把输出夹在 [-output_max, +output_max]）
} PID_t;

// 对传入的PID结构体初始化一套PID系数以及限制最大输出
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output);
// PID计算函数；分别传入期望值、反馈值
float PID_Calc(PID_t *pid, float ref, float feedback);

#endif
