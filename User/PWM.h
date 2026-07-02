#pragma once
#include "main.h"

typedef enum
{
    PWM_CH_LEFT = 0,
    PWM_CH_RIGHT,
    PWM_CH_NUM,
}PWM_Channel_t;

static const PWM_Channel_t PWM_CHANNELS[PWM_CH_NUM]=
{[PWM_CH_LEFT] = {&htim1, TIM_CHANNEL_1}, 
 [PWM_CH_RIGHT] = {&htim1, TIM_CHANNEL_2},
};
void Set_PWM_Duty(PWM_Channel_t channel, uint16_t duty);
