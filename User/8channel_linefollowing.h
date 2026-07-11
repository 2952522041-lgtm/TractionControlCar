#pragma once

#include "main.h"

#define LF_SENSOR_NUM 8U

/* x1 is the left-most sensor and x8 is the right-most sensor. */
#define LINEFOLLOWING_ACTIVE_VALUE 0U
#define LINEFOLLOWING_BASE_RPM 80.0f
#define LINEFOLLOWING_TURN_PARAMETER 20.0f
#define LINEFOLLOWING_TIMEOUT_MS 100U
#define LINEFOLLOWING_UART_TIMEOUT_MS 100U
#define LINEFOLLOWING_DATAPACKET_LENGTH 80U

/* Keep the old misspelled name compatible with existing code. */
#define LINEFOLLOWING_DATAPACKET_LENTGH LINEFOLLOWING_DATAPACKET_LENGTH

typedef struct
{
    uint8_t value[LF_SENSOR_NUM];
    uint32_t tick_ms;
    uint8_t valid;
} LineFollow_SensorData_t;

typedef struct
{
    float left_target_rpm;
    float right_target_rpm;
    uint32_t tick_ms;
    uint8_t valid;
} LineFollow_targetRPM_t;

void LineFollow_Init(void);
void LineFollow_InputByte(uint8_t byte);
void LineFollow_Processdata(void);

void LineFollow_GetTargetRPM(LineFollow_targetRPM_t *target);
void LineFollow_GetSensorData(LineFollow_SensorData_t *data);
