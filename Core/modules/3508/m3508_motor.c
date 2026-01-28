#include "m3508_motor.h"

/* -------------------- 初始化所有电机 -------------------- */
void M3508_InitAll(M3508_t *motors, CAN_HandleTypeDef *hcan)
{
    // *motors传入一个电机数组包含要控制的所有电机 hcan传入所用的CAN口号
    // 按照最大数量遍历数组
    for (int i = 0; i < M3508_MAX_NUM; i++)
    {
        // 设置每个电机的接收ID
        uint32_t rx_id = 0x201 + i;

        // 组装 CAN 注册配置
        CAN_Init_Config_s config = {
            .can_handle = hcan,  // 具体CAN口
            .tx_id = 0x200,  // 发送指令标识符（这里只能控制4个电机，从第5个开始标识符要改为0x1ff）
            .rx_id = rx_id,  // 这个电机自己的反馈帧 ID
            .can_module_callback = M3508_Callback,  // 当收到对应 rx_id 时就调用这个回调解析
            .id = &motors[i]
        };

        motors[i].can = CANRegister(&config);  // 注册 CAN 实例，返回 CANInstance* 保存到电机对象里
        motors[i].id = i + 1; // 电机编号（1~4）
        motors[i].target_speed = 0;  // 初始化目标转速为0
        // 初始化PID（此时所有电机共用一套PID）（速度环PID）
        // 后续可以在外部访问motor数组的pid进行单独对应更改
        // 这里max_output是返回的最大电流值，官方手册中3508的限制是正负16384
        PID_Init(&motors[i].pid, 1.2f, 0.0f, 0.05f, 10000.0f);
        CANSetDLC(motors[i].can, 8);  // 设置发送帧长度为 8 字节
    }
}

/* -------------------- 设置目标转速 -------------------- */
void M3508_SetTarget(M3508_t *motor, float target_rpm)
{
    // 仅仅把目标速度写入电机对象
    // 此时只是传入目标，没有执行，后续Update中会统一发送执行
    motor->target_speed = target_rpm;
}

/* -------------------- PID计算并统一发送 -------------------- */
void M3508_UpdateAll(M3508_t *motors, uint8_t motor_count)
{
    // 对前 motor_count 个电机做控制计算并发送
    // 这里最多4个电机

    // 准备一个长度为 4 的电流指令数组（对应 4 个电机）
    int16_t currents[4] = {0};
    // 遍历每一个电机
    for (int i = 0; i < motor_count; i++)
    {
        // 计算PID（分别传入对应电机的PID参数，目标转速，电机当前转速）
        // 返回一个电流指令值
        float out = PID_Calc(&motors[i].pid,
                             motors[i].target_speed,
                             motors[i].feedback.speed_rpm);
        // 再一次判断，限制返回的电流指令值过大
        if (out > 10000) out = 10000;
        if (out < -10000) out = -10000;
        // 将计算后的电流值存入currents数组
        currents[i] = (int16_t)out;
    }

    // 打包发送 0x200 帧
    motors[0].can->tx_buff[0] = (currents[0] >> 8) & 0xFF;
    motors[0].can->tx_buff[1] = (currents[0]) & 0xFF;
    motors[0].can->tx_buff[2] = (currents[1] >> 8) & 0xFF;
    motors[0].can->tx_buff[3] = (currents[1]) & 0xFF;
    motors[0].can->tx_buff[4] = (currents[2] >> 8) & 0xFF;
    motors[0].can->tx_buff[5] = (currents[2]) & 0xFF;
    motors[0].can->tx_buff[6] = (currents[3] >> 8) & 0xFF;
    motors[0].can->tx_buff[7] = (currents[3]) & 0xFF;

    // 发送CAN帧
    CANTransmit(motors[0].can, 2);
}

/* -------------------- CAN反馈回调 -------------------- */
void M3508_Callback(CANInstance *instance)
{
    M3508_t *motor = (M3508_t *)instance->id;
    uint8_t *d = instance->rx_buff;

    motor->feedback.rotor_angle   = (uint16_t)((d[0] << 8) | d[1]);
    motor->feedback.speed_rpm     = (int16_t)((d[2] << 8) | d[3]);
    motor->feedback.given_current = (int16_t)((d[4] << 8) | d[5]);
    motor->feedback.temp          = d[6];
}
