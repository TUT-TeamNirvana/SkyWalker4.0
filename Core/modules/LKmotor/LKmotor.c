#include "LKmotor.h"
#include <string.h>

LKMotor_t lkmotor[LKMOTOR_CNT + 1];

static void LKMotor_RegisterOne(uint8_t id)
{
    CAN_Init_Config_s config;

    config.can_handle = &hcan1; // 你工程里默认只有一个 hcan
    config.tx_id = LKMOTOR_BASE_ID + id;
    config.rx_id = LKMOTOR_BASE_ID + id;
    config.can_module_callback = LKMotor_CANCallback;
    config.id = (void*)(uintptr_t)id;   // 用于区分电机

    lkmotor[id].can = CANRegister(&config);
}

void LKMotor_InitAll(void)
{
    for (int i = 1; i <= LKMOTOR_CNT; i++)
    {
        memset(&lkmotor[i], 0, sizeof(LKMotor_t));
        LKMotor_RegisterOne(i);
    }
}

/***********************************************
 * 核心控制命令：多圈位置闭环控制命令 2 (0xA4)
 ***********************************************/
void LKMotor_SetPositionLoop2(uint8_t id, int32_t angleControl, uint16_t maxSpeed)
{
    if (id < 1 || id > LKMOTOR_CNT) return;

    CANInstance *can = lkmotor[id].can;

    CANSetDLC(can, 8);

    can->tx_buff[0] = LKMOTOR_CMD_POS_LOOP2;
    can->tx_buff[1] = 0x00;

    memcpy(&can->tx_buff[2], &maxSpeed, 2);
    memcpy(&can->tx_buff[4], &angleControl, 4);

    CANTransmit(can, 1);
}

/***********************************************
 *  CAN 回调：自动由 bsp_can 调用
 ***********************************************/
void LKMotor_CANCallback(CANInstance *instance)
{
    uint8_t id = (uint8_t)(uintptr_t)instance->id;

    LKMotor_t *m = &lkmotor[id];

    uint8_t *d = instance->rx_buff;

    m->state   = d[1];
    m->current = (int16_t)(d[3] << 8 | d[2]);
    m->speed   = (int16_t)(d[5] << 8 | d[4]);
    m->encoder_raw = (int16_t)(d[7] << 8 | d[6]);

    if (!m->inited)
    {
        m->inited = 1;
        m->last_encoder_raw = m->encoder_raw;
        m->multi_turn_pos = m->encoder_raw;
        return;
    }

    int16_t diff = m->encoder_raw - m->last_encoder_raw;

    if (diff > 30000) diff -= 65536;
    else if (diff < -30000) diff += 65536;

    m->multi_turn_pos += diff;
    m->last_encoder_raw = m->encoder_raw;
}

