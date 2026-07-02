#include "PWM.h"
#include "tim.h"

void Set_PWM_Duty(PWM_Channel_t channel, uint16_t duty)
{
    TIM_HandleTypeDef *htim = PWM_CHANNELS[channel].htim;
    uint32_t tim_channel = PWM_CHANNELS[channel].tim_channel;

    __HAL_TIM_SET_COMPARE(htim, tim_channel, duty);
}