//
// Created by 24699 on 26-2-6.
//

#include "ArmControl.h"

int   G_STATE = 1;        // 夹爪状态
float G_JOINT_DEG[6]; // 机械臂关节角度

// 融合版：收到一行就解析 S 帧并更新全局数据
void UART6_OnLine(char *line)
{
    // S,0,23.6,-31.2,38.4,-29.0,-32.6,76.7
    int st = 0;
    float j[6];
    char buf[UART6_DMA_RX_SIZE];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    // 去掉末尾 \r \n
    size_t n = strlen(buf);
    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[n - 1] = '\0';
        n--;
    }
    // strtok_r 开始解析
    char *saveptr = NULL;
    char *tok = strtok_r(buf, ",", &saveptr);
    if (!tok) return;
    // 第 1 段：必须是 "S"
    if (!(tok[0] == 'S' && tok[1] == '\0')) return;
    // 第 2 段：state（整数）
    tok = strtok_r(NULL, ",", &saveptr);
    if (!tok) return;
    st = (int)strtol(tok, NULL, 10);
    // 第 3~8 段：6 个 float
    for (int i = 0; i < 6; i++) {
        tok = strtok_r(NULL, ",", &saveptr);
        if (!tok) return;

        char *endp = NULL;
        float v = strtof(tok, &endp);
        if (endp == tok) return;     // 解析失败（没有任何数字被转换）
        j[i] = v;
    }
    // 解析成功：统一写入全局
    G_STATE = st;
    for (int i = 0; i < 6; i++) {
        G_JOINT_DEG[i] = j[i];
    }
}

// 机械臂初始化
void ArmInit(ArmControl *Arm, CAN_HandleTypeDef *hcan) {

    BSP_UART6_Init();
    UART6_RegisterLineCallback(UART6_OnLine);
    LKMG_InitAll(Arm->arm_motors, hcan);

    // 设置减速比
    Arm->arm_motors[5].Speed_Ratio = 10;
    Arm->arm_motors[4].Speed_Ratio = 10;
    Arm->arm_motors[3].Speed_Ratio = 6;
    Arm->arm_motors[2].Speed_Ratio = 36;
    Arm->arm_motors[1].Speed_Ratio = 36;
    Arm->arm_motors[0].Speed_Ratio = 6;

    LKMG_SetCurrent(&Arm->arm_motors[6], 50);
}

// 机械臂角度更新并控制
void ArmUpdate(ArmControl *Arm) {

    UART6_Print("Get %d ", Arm->g_state);
    if (Arm->g_state == 0 && G_STATE == 1) {
        LKMG_SetCurrent(&Arm->arm_motors[6], 50);
        Arm->g_state = 1;
        LKMG_CurrentControl_each(&Arm->arm_motors[6]);
    }
    else if (Arm->g_state == 1 && G_STATE == 0) {
        LKMG_SetCurrent(&Arm->arm_motors[6], -50);
        Arm->g_state = 0;
        LKMG_CurrentControl_each(&Arm->arm_motors[6]);
    }

    for (int i=0;i<6;i++) {

        Arm->g_joint_deg[i] = G_JOINT_DEG[i];
        Arm->g_state = G_STATE;

        LKMG_SetPos(&Arm->arm_motors[i], Arm->g_joint_deg[i] * 100 * Arm->arm_motors[i].Speed_Ratio);

        UART6_Print("%d ", (int)(Arm->g_joint_deg[i]*10));
    }

    UART6_Print("\n");
    LKMG_PosControl(Arm->arm_motors, 6);
}
