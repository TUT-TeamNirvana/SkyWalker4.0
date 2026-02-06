//
// Created by 24699 on 26-2-2.
//

#ifndef LK_MG_MOTOR_H
#define LK_MG_MOTOR_H

#include "bsp_can.h"
#include "bsp_log.h"

#define LKMG_MAX_NUM 6  // 挂载电机的最大数量
#define MAX_CURRENT 1240  // 最大电流

// 反馈数据结构体
typedef struct
{
    int8_t temp;  // 电机反馈温度
    int16_t given_current;  // 电机反馈电流
    int16_t speed_rpm;  //电机反馈转速
    uint16_t rotor_angle;  // 电机转子的机械角度
} LKMG_Feedback_t;

// 电机对象结构体
typedef struct
{
    CANInstance *can;  // 该电机对应的 CAN 实例（通过 CANRegister 注册后得到）
    LKMG_Feedback_t feedback;  // 缓存最新一次 CAN 回调解析出来的反馈数据（代表电机当前状态）
    int16_t target_current;  // 设置目标电流
    int32_t target_speed;  // 目标转速
    int32_t target_pos;  // 目标位置
    int16_t max_speed;  // 设置最大速度
    uint8_t id;  // 电机编号
    float Speed_Ratio;  // 电机减速比
} LKMG_t;

// 注意翎控MG系列转矩环不是电流环但这里先当电流环用
void LKMG_InitAll(LKMG_t *motors, CAN_HandleTypeDef *hcan);  // 初始化数组里所有电机对象，并注册 CAN
void LKMG_GetInfo(LKMG_t *motors);  // 获取数据
void LKMG_SetCurrent(LKMG_t *motor, float target_current);  // 设置单个电机目标电流
void LKMG_SetSpeed(LKMG_t *motor, float target_speed);  // 设置转速
void LKMG_SetPos(LKMG_t *motor, float target_pos);  // 设置目标位置
void LKMG_CurrentControl(LKMG_t *motors);  // 电流环控制
void LKMG_SpeedControl(LKMG_t *motors);  // 速度环
void LKMG_PosControl(LKMG_t *motors);  // 位置环
void LKMG_Callback(CANInstance *instance);  // CAN 接收回调
void LKMG_LogShow(LKMG_t *motor);  // rtt 调试显示

#endif //LK_MG_MOTOR_H
