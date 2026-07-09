#pragma once
#include "main.h"

#define LF_SENSOR_NUM 8
#define LINEFOLLOWING_TURN_PARAMETER 0.0f
#define LINEFOLLOWING_TIMEOUT_MS 100U
#define LINEFOLLOWING_DATAPACKET_LENTGH 80U

typedef struct
{
    uint8_t value[LF_SENSOR_NUM];
    uint32_t tick_ms;
    uint8_t valid;
}LineFollow_SensorData_t;

typedef struct
{
    float left_target_rpm;
    float right_target_rpm;
    uint32_t tick_ms;
    uint8_t valid;
}LineFollow_targetRPM_t;

void LineFollow_Init(void);
void LineFollow_InputByte(uint8_t byte);
void LineFollow_Processdata(void);

void LineFollow_GetTargetRPM(LineFollow_target_t *target);
void LineFollow_GetSensorData(LineFollow_SensorData_t *data);