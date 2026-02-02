//
// Created by 24699 on 26-2-2.
//

#ifndef LK_MG_MOTOR_H
#define LK_MG_MOTOR_H

#include "bsp_can.h"

#define M3508_MAX_NUM 4  // 挂载3508的最大数量

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
    uint8_t id;  // 电机编号
} LKMG_t;

// 注意翎控MG系列转矩环不是电流环但这里先当电流环用
void LKMG_InitAll(LKMG_t *motors, CAN_HandleTypeDef *hcan);  // 初始化数组里所有电机对象，并注册 CAN
void LKMG_SetCurrent(LKMG_t *motor, float target_current);  // 设置单个电机目标电流
void LKMG_CurrentControl(LKMG_t *motors);  // 电流环控制
void LKMG_Callback(CANInstance *instance);  // CAN 接收回调

#endif //LK_MG_MOTOR_H
