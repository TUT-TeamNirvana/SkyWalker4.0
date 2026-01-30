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
        // 初始化位置环PID参数
        PosPID_Init(&motors[i].pos_pid, 0.5f, 0.0f, 0.0f, 6000.0f);
        CANSetDLC(motors[i].can, 8);  // 设置发送帧长度为 8 字节
        // 初始化电机位置环各个参数
        motors[i].position_ticks = 0;  // 多圈位置归零
        motors[i]._last_raw_angle = 0;  // 上一次的角度归零
        motors[i]._angle_inited = 0;  // 初始化标志
    }
}

// 更新电机多圈位置 传入当先电机直接反馈的编码器的值
static void M3508_UpdateMultiTurn(M3508_t *motor, uint16_t raw){
    // 初始化判断
    if (!motor->_angle_inited){
        motor->_angle_inited = 1;  // 初始化标志设置1
        motor->_last_raw_angle = raw;  // 保存当前角度
        motor->position_ticks = 0;  // 初始化之后的状态为0点
        return;
    }
    // 计算当前与之前的角度差值
    int32_t diff = (int32_t)raw - (int32_t)motor->_last_raw_angle;
    // 8192 ticks/turn，跨零处理阈值 4096
    if (diff > 4096)       diff -= 8192;
    else if (diff < -4096) diff += 8192;

    motor->position_ticks += diff;
    motor->_last_raw_angle = raw;
}

// 设置目标电流
void M3508_SetCurrent(M3508_t *motor, float target_current) {
    // 范围限定
    if (target_current > 10000) target_current = 10000;
    if (target_current < -10000) target_current = -10000;
    motor->target_current = (int16_t)target_current;
}
// 设置目标转速
void M3508_SetSpeed(M3508_t *motor, float target_rpm){
    motor->target_speed = target_rpm;
}
// 电机电流环直控
void M3508_CurrentControl(M3508_t *motors) {

    motors[0].can->tx_buff[0] = (motors[0].target_current >> 8) & 0xFF;
    motors[0].can->tx_buff[1] = (motors[0].target_current) & 0xFF;
    motors[0].can->tx_buff[2] = (motors[1].target_current >> 8) & 0xFF;
    motors[0].can->tx_buff[3] = (motors[1].target_current) & 0xFF;
    motors[0].can->tx_buff[4] = (motors[2].target_current >> 8) & 0xFF;
    motors[0].can->tx_buff[5] = (motors[2].target_current) & 0xFF;
    motors[0].can->tx_buff[6] = (motors[3].target_current >> 8) & 0xFF;
    motors[0].can->tx_buff[7] = (motors[3].target_current) & 0xFF;

    CANTransmit(motors[0].can, 2);
}
// 速度环PID计算并控制
void M3508_SpeedControl(M3508_t *motors, uint8_t motor_count)
{
    // 遍历每一个电机
    for (int i = 0; i < motor_count; i++)
    {
        // 计算PID（分别传入对应电机的PID参数，目标转速，电机当前转速）
        // 返回一个电流指令值
        float out = PID_Calc(&motors[i].pid,
                             motors[i].target_speed,
                             motors[i].feedback.speed_rpm);
        // 将计算后的电流值进行设定
        M3508_SetCurrent(&motors[i], out);
    }
    M3508_CurrentControl(motors);
}

// 电机位置环归零
void M3508_ResetPosition(M3508_t *motor)
{
    motor->position_ticks = 0;  // 多圈位置
    motor->_last_raw_angle = 0;  // 上一次位置
    motor->_angle_inited = 0;  // 初始化标志
}

/* -------------------- CAN反馈回调 -------------------- */
void M3508_Callback(CANInstance *instance)
{
    // 找到指定ID的电机
    M3508_t *motor = (M3508_t *)instance->id;
    uint8_t *d = instance->rx_buff;

    // 按照手册分别获取指定的数据
    motor->feedback.rotor_angle   = (uint16_t)((d[0] << 8) | d[1]);
    motor->feedback.speed_rpm     = (int16_t)((d[2] << 8) | d[3]);
    motor->feedback.given_current = (int16_t)((d[4] << 8) | d[5]);
    motor->feedback.temp          = d[6];
    // 更新电机多圈位置
    M3508_UpdateMultiTurn(motor, motor->feedback.rotor_angle);
}

// rtt 速度波形调试显示
void Speed_LogShow(M3508_t *motor) {
    // 分别显示目标转速和实际转速的波形
    float target_speed = motor->target_speed;
    float speed_rpm = motor->feedback.speed_rpm;
    RTT_PrintWave_vofa(2, target_speed, speed_rpm);
}
