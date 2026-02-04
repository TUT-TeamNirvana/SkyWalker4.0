#include "lk-mg_motor.h"

/* -------------------- 初始化所有电机 -------------------- */
void LKMG_InitAll(LKMG_t *motors, CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < LKMG_MAX_NUM; i++)
    {
        // 设置每个电机的接收ID
        uint32_t tx_id = 0x141 + i;
        uint32_t rx_id = 0x141 + i;

        // 组装 CAN 注册配置
        CAN_Init_Config_s config = {
            .can_handle = hcan,  // 具体CAN口
            .tx_id = tx_id,  // 发送指令标识符
            .rx_id = rx_id,  // 这个电机自己的反馈帧 ID
            .can_module_callback = LKMG_Callback,  // 当收到对应 rx_id 时就调用这个回调解析
            .id = &motors[i]
        };

        motors[i].can = CANRegister(&config);  // 注册 CAN 实例，返回 CANInstance* 保存到电机对象里
        motors[i].id = i + 1; // 电机编号

        CANSetDLC(motors[i].can, 8);  // 设置发送帧长度为 8 字节

        motors[i].target_current = 0;
        motors[i].target_speed = 0;
        motors[i].target_pos = 0;
        motors[i].max_speed = 3600;
    }
}

// 获取数据
void LKMG_GetInfo(LKMG_t *motors) {
    for (int i = 0; i < LKMG_MAX_NUM; i++) {
        motors[i].can->tx_buff[0] = 0x9C;
        for (int j=1;j<=7;j++)  motors[i].can->tx_buff[j] = 0;
        CANTransmit(motors[i].can, 2);
    }
}

// 设置目标电流
void LKMG_SetCurrent(LKMG_t *motor, float target_current) {
    // 范围限定
    if (target_current > MAX_CURRENT) target_current = MAX_CURRENT;
    if (target_current < -MAX_CURRENT) target_current = -MAX_CURRENT;
    motor->target_current = (int16_t)target_current;
}
// 设置目标转速
void LKMG_SetSpeed(LKMG_t *motor, float target_speed) {
    motor->target_speed = (int32_t)target_speed;
}
// 设置目标位置
void LKMG_SetPos(LKMG_t *motor, float target_pos) {
    motor->target_pos = (int32_t)target_pos;
}

// 转矩环
void LKMG_CurrentControl(LKMG_t *motors) {

    for (int i = 0; i < LKMG_MAX_NUM; i++) {
        if (motors[i].feedback.given_current == motors[i].target_current)
            continue;

        motors[i].can->tx_buff[0] = 0xA1;
        motors[i].can->tx_buff[1] = 0;
        motors[i].can->tx_buff[2] = 0;
        motors[i].can->tx_buff[3] = 0;
        motors[i].can->tx_buff[4] = (motors[i].target_current) & 0xFF;
        motors[i].can->tx_buff[5] = (motors[i].target_current >> 8) & 0xFF;
        motors[i].can->tx_buff[6] = 0;
        motors[i].can->tx_buff[7] = 0;

        CANTransmit(motors[i].can, 2);
    }
}
// 速度环
void LKMG_SpeedControl(LKMG_t *motors) {

    for (int i = 0; i < LKMG_MAX_NUM; i++) {
        if (motors[i].feedback.speed_rpm == motors[i].target_speed)
            continue;

        int16_t max_current = MAX_CURRENT;

        motors[i].can->tx_buff[0] = 0xA2;
        motors[i].can->tx_buff[1] = 0;
        motors[i].can->tx_buff[2] = max_current & 0xFF;
        motors[i].can->tx_buff[3] = (max_current >> 8) & 0xFF;
        motors[i].can->tx_buff[4] = (motors[i].target_speed) & 0xFF;
        motors[i].can->tx_buff[5] = (motors[i].target_speed >> 8) & 0xFF;
        motors[i].can->tx_buff[6] = (motors[i].target_speed >> 16) & 0xFF;
        motors[i].can->tx_buff[7] = (motors[i].target_speed >> 24) & 0xFF;

        CANTransmit(motors[i].can, 2);
    }
}
// 位置环
void LKMG_PosControl(LKMG_t *motors) {

    for (int i = 0; i < LKMG_MAX_NUM; i++) {
        //if (motors[i].feedback.rotor_angle == motors[i].target_pos)
            //continue;

        int16_t max_speed = (int16_t)motors[i].max_speed;

        motors[i].can->tx_buff[0] = 0xA4;
        motors[i].can->tx_buff[1] = 0;
        motors[i].can->tx_buff[2] = max_speed & 0xFF;
        motors[i].can->tx_buff[3] = (max_speed >> 8) & 0xFF;
        motors[i].can->tx_buff[4] = (motors[i].target_pos) & 0xFF;
        motors[i].can->tx_buff[5] = (motors[i].target_pos >> 8) & 0xFF;
        motors[i].can->tx_buff[6] = (motors[i].target_pos >> 16) & 0xFF;
        motors[i].can->tx_buff[7] = (motors[i].target_pos >> 24) & 0xFF;

        CANTransmit(motors[i].can, 2);
    }
}

/* -------------------- CAN反馈回调 -------------------- */
void LKMG_Callback(CANInstance *instance)
{
    // 找到指定ID的电机
    LKMG_t *motor = (LKMG_t *)instance->id;
    uint8_t *d = instance->rx_buff;

    // 按照手册分别获取指定的数据
    motor->feedback.temp          = (int8_t)d[1];
    motor->feedback.given_current = (int16_t)(d[2] | (d[3] << 8));
    motor->feedback.speed_rpm     = (int16_t)(d[4] | (d[5] << 8));
    motor->feedback.rotor_angle   = (uint16_t)(d[6] | (d[7] << 8));
}

// rtt 调试显示
void LKMG_LogShow(LKMG_t *motor) {
    float temp = motor->feedback.temp;  // 电机反馈温度
    float given_current = motor->feedback.given_current;  // 电机反馈电流
    float speed_rpm = motor->feedback.speed_rpm;  //电机反馈转速
    float rotor_angle = motor->feedback.rotor_angle;  // 电机转子的机械角度
    RTT_PrintWave_vofa(4, temp, given_current, speed_rpm, rotor_angle);
}
