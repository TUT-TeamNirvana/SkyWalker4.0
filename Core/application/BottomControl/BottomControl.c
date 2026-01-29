#include "BottomControl.h"

// 底盘初始化
void BottomInit(BottomControl *Bottom) {

    M3508_InitAll(Bottom->bottom_motors, &hcan1);
    // 设置底盘电机速度环和位置环pid的时间间隔
    for (int i=0;i<4;i++) {
        PID_SetDt(&Bottom->bottom_motors[i].pid, SPEED_LOOP_DT);
        PosPID_SetDt(&Bottom->bottom_motors[i].pos_pid, POS_LOOP_DT);
    }
    // 静止运动状态
    Bottom->vx = Bottom->vy = Bottom->wz = 0;
    // 电机安装方向
    Bottom->dir[0] = +1;
    Bottom->dir[1] = +1;
    Bottom->dir[2] = -1;
    Bottom->dir[3] = -1;
}

// 映射函数
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 福斯i6输入形式的遥感通道映射
void SbusI6Mode(BottomControl *Bottom, int16_t CHX, int16_t CHY, int16_t CHZ) {

    Bottom->vx = mapf(CHX, 240, 1800, -8000, +8000);  // 前后
    Bottom->vy = mapf(CHY, 240, 1800, +8000, -8000);  // 左右
    Bottom->wz = mapf(CHZ, 240, 1800, +8000, -8000);  // 旋转
    // i6的摇杆死区
    if (fabs(Bottom->vx) < 200) Bottom->vx = 0;
    if (fabs(Bottom->vy) < 200) Bottom->vy = 0;
    if (fabs(Bottom->wz) < 200) Bottom->wz = 0;
}

// 底盘运动控制及数据更新(麦克拉姆形式解算)
void Chassis_Control(BottomControl *Bottom){
    // 麦克纳姆轮运动学分配 (X形布局)
    float v1 = +Bottom->vx - Bottom->vy - Bottom->wz;  // 左前
    float v2 = +Bottom->vx + Bottom->vy - Bottom->wz;  // 左后
    float v3 = +Bottom->vx - Bottom->vy + Bottom->wz;  // 右后
    float v4 = +Bottom->vx + Bottom->vy + Bottom->wz;  // 右前
    // 判断是否静止
    if (v1 == v2 && v2 == v3 && v3 == v4 && v4 == 0) {
        // 遍历每一个电机
        for (int i=0;i<4;i++) {
            PosPID_SetRef(&Bottom->bottom_motors[i].pos_pid, 0);  // 直接设定目标位置为零点
            int32_t pos_now = M3508_GetPositionTicks(&Bottom->bottom_motors[i]);  // 获取当前多圈累加后的位置
            float speed_ref = PosPID_Calc(&Bottom->bottom_motors[i].pos_pid, pos_now);  // 位置环计算输出速度
            M3508_SetSpeed(&Bottom->bottom_motors[i], speed_ref);  // 将计算好的速度传给速度环
        }
        return;
    }
    // 如果没有静止 则不断刷新零点
    for (int i=0;i<4;i++) {
        M3508_ResetPosition(&Bottom->bottom_motors[i]);
    }
    // 应用安装方向表
    M3508_SetSpeed(&Bottom->bottom_motors[0], Bottom->dir[0] * v1);
    M3508_SetSpeed(&Bottom->bottom_motors[1], Bottom->dir[1] * v2);
    M3508_SetSpeed(&Bottom->bottom_motors[2], Bottom->dir[2] * v3);
    M3508_SetSpeed(&Bottom->bottom_motors[3], Bottom->dir[3] * v4);
}

// 底盘解算并更新发送执行
void BottomUpdate(BottomControl *Bottom) {
    Chassis_Control(Bottom);
    M3508_SpeedControl(Bottom->bottom_motors, 4);
}
