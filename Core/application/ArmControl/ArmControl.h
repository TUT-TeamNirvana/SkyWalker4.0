//
// Created by 24699 on 26-2-6.
//

#ifndef ARMCONTROL_H
#define ARMCONTROL_H

#include "bsp_uart6.h"
#include "lk-mg_motor.h"

typedef struct
{
    LKMG_t arm_motors[6];  // 电机
    int   g_state;        // 夹爪状态
    float g_joint_deg[6]; // 机械臂关节角度
} ArmControl;

void ArmInit(ArmControl *Arm, CAN_HandleTypeDef *hcan);  // 机械臂初始换
void UART6_OnLine(char *line);  // 串口解析函数
void ArmUpdate(ArmControl *Arm);  // 机械臂数据更新并控制

#endif //ARMCONTROL_H
