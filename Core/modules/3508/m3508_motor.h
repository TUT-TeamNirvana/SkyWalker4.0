#ifndef M3508_MOTOR_H
#define M3508_MOTOR_H

#include "main.h"
#include "bsp_can.h"
#include "pid.h"
#include "pos_pid.h"
#include "bsp_log.h"

#define M3508_MAX_NUM 4  // 挂载3508的最大数量

// 反馈数据结构体
typedef struct
{
    uint16_t rotor_angle;  // 电机转子的机械角度
    int16_t speed_rpm;  //电机反馈转速
    int16_t given_current;  // 电机反馈电流
    uint8_t temp;  // 电机反馈温度
} M3508_Feedback_t;

// 电机对象结构体
typedef struct
{
    CANInstance *can;  // 该电机对应的 CAN 实例（通过 CANRegister 注册后得到）
    M3508_Feedback_t feedback;  // 缓存最新一次 CAN 回调解析出来的反馈数据（代表电机当前状态）
    PID_t pid;  // 速度环 PID 控制器实例
    PosPID_t pos_pid;  // 位置环PID
    int16_t target_current;  // 设置目标电流
    float target_speed;  // 目标转速（rpm）
    uint8_t id;  // 电机编号

    int32_t position_ticks;      // 连续多圈位置（每转+8192）
    uint16_t _last_raw_angle;    // 上次原始角度(0~8191)
    uint8_t  _angle_inited;      // 初始化标志
} M3508_t;

void M3508_InitAll(M3508_t *motors, CAN_HandleTypeDef *hcan);  // 初始化数组里所有电机对象，并注册 CAN
void M3508_SetCurrent(M3508_t *motor, float target_current);  // 设置单个电机目标电流
void M3508_SetSpeed(M3508_t *motor, float target_rpm);  // 设置单个电机目标转速
void M3508_CurrentControl(M3508_t *motors);  // 全部电机通过0x200统一发送电流命令
void M3508_SpeedControl(M3508_t *motors, uint8_t motor_count);  // 对所有电机做一次电流环PID计算并发送电流
void M3508_Callback(CANInstance *instance);  // CAN 接收回调，用于解析反馈帧（由 bsp_can 收到对应 ID 时调用）
static int32_t M3508_GetPositionTicks(const M3508_t *motor){  // 返回连续多圈位置值
    return motor->position_ticks;
}
void M3508_ResetPosition(M3508_t *m);  // 位置归零
void Speed_LogShow(M3508_t *motor);  // rtt 速度波形调试显示

#endif
