#ifndef __LK_MOTOR_H
#define __LK_MOTOR_H

#include "bsp_can.h"
#include "stdint.h"
#include "usb.h"

#define LKMOTOR_CNT 6
#define LKMOTOR_BASE_ID 0x140
#define LKMOTOR_CMD_POS_LOOP2 0xA4

typedef struct
{
    CANInstance *can;        // 绑定的CAN实例
    int32_t encoder_raw;     // 原始单圈编码器
    int64_t multi_turn_pos;  // 多圈累计角度
    int32_t last_encoder_raw;
    uint8_t inited;

    int16_t speed;
    int16_t current;
    uint8_t state;
} LKMotor_t;

extern LKMotor_t lkmotor[LKMOTOR_CNT + 1];

void LKMotor_InitAll(void);
void LKMotor_CANCallback(CANInstance *instance);

void LKMotor_SetPositionLoop2(uint8_t id, int32_t angleControl, uint16_t maxSpeed);

#endif
