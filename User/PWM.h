#pragma once
#include "main.h"
#include "tim.h"

#define PWM_MAX_DUTY 3599u

typedef enum
{
    PWM_CH_LEFT = 0,
    PWM_CH_RIGHT,
    PWM_CH_NUM,
}PWM_Channel_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} PWM_Config_t;

void Set_PWM_Duty(PWM_Channel_t channel, uint16_t duty);
