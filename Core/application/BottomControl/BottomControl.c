#include "BottomControl.h"

// 底盘初始化
void BottomInit(BottomControl *Bottom) {

    M3508_InitAll(Bottom->bottom_motor, &hcan1);
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
    // 应用安装方向表
    M3508_SetSpeed(&Bottom->bottom_motor[0], Bottom->dir[0] * v1);
    M3508_SetSpeed(&Bottom->bottom_motor[1], Bottom->dir[1] * v2);
    M3508_SetSpeed(&Bottom->bottom_motor[2], Bottom->dir[2] * v3);
    M3508_SetSpeed(&Bottom->bottom_motor[3], Bottom->dir[3] * v4);
}

// 底盘解算并更新发送执行
void BottomUpdate(BottomControl *Bottom) {
    Chassis_Control(Bottom);
    M3508_SpeedControl(Bottom->bottom_motor, 4);
}
