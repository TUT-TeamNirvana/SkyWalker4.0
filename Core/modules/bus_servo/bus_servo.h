#pragma once
#include <stdint.h>

void BusServo_Move2(uint16_t time_ms, uint16_t pos1, uint16_t pos2);
uint16_t BusServo_AngleToPos(float deg,
                             float deg_min, float deg_max,
                             uint16_t pos_min, uint16_t pos_max);
