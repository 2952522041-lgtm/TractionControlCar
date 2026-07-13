#pragma once
#include "main.h"

#define LF_SENSOR_NUM 8
#define LINEFOLLOWING_TURN_PARAMETER 5.0f//还未调试
#define LINEFOLLOWING_TIMEOUT_MS 100U
#define LINEFOLLOWING_DATAPACKET_LENTGH 80U
#define LF_CarBaseRPM 160.0f
#define LF_CarMAXRPM 330.0f
#define LF_CarMINRPM 0.0f


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
void LineFollow_StartUartReceive(void); 
void LineFollow_UartRxCpltCallback(UART_HandleTypeDef *huart);

void LineFollow_InputByte(uint8_t byte);
void LineFollow_Update(void);

uint8_t LineFollow_GetTargetRPM(LineFollow_targetRPM_t *target);
uint8_t LineFollow_GetSensorData(LineFollow_SensorData_t *data);
uint8_t LineFollow_IsTargetTimeout(uint32_t now_ms);