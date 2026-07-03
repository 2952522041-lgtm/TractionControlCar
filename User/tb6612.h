#pragma once
#include "main.h"

#define tb6612_PWM_MAX_DUTY 3599u
#define tb6612_rpm_direction 1.0f
#define tb6612_max_rpm 100.0f

typedef enum
{
    tb6612_DIR_STOP = 0,
    tb6612_DIR_FORWARD,
    tb6612_DIR_BACKWARD,
    tb6612_DIR_BRAKE
}tb6612_Direction_t;

typedef enum
{
    tb6612_CH_LEFT = 0,
    tb6612_CH_RIGHT,
    tb6612_CH_NUM,

}tb6612_Channel_t;

void tb6612_Init(void);
void tb6612_SetDuty(tb6612_Channel_t tb6612_Channel, uint32_t duty);
void tb6612_SetDirection(tb6612_Channel_t tb6612_Channel, tb6612_Direction_t direction);
void tb6612_Stop(tb6612_Channel_t tb6612_Channel);
void tb6612_Brake(tb6612_Channel_t tb6612_Channel);
void tb6612_SetRPM(tb6612_Channel_t tb6612_Channel, float rpm);
