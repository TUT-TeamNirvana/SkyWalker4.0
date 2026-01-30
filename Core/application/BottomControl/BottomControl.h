//
// Created by 24699 on 26-1-29.
//

#ifndef BUTTONCONTROL_H
#define BUTTONCONTROL_H

#include "m3508_motor.h"
#include <math.h>

// 底盘电机串级PID的时间间隔
#define SPEED_LOOP_DT 0.001f
#define POS_LOOP_DT 0.001f

typedef struct
{
    M3508_t bottom_motors[4];  // 4个电机
    int8_t dir[4]; // 按照电机安装方向
    float vx, wz, vy;  // 前后 左右 旋转
} BottomControl;

void BottomInit(BottomControl *Bottom);  // 底盘初始化
void SbusI6Mode(BottomControl *Bottom, int16_t CHX, int16_t CHY, int16_t CHZ);  // 福斯i6输入形式的遥感通道映射
void BottomUpdate(BottomControl *Bottom);  // 底盘解算并更新发送执行
void BottomMotorSpeedlog(BottomControl *Bottom, int number);  // 底盘电机转速调试波形

#endif //BUTTONCONTROL_H
