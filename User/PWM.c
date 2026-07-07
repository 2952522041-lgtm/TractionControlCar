#include "PWM.h"
#include "tim.h"

static const PWM_Config_t PWM_CHANNELS[PWM_CH_NUM] =
{
    [PWM_CH_LEFT] = {&htim1, TIM_CHANNEL_1},
    [PWM_CH_RIGHT] = {&htim1, TIM_CHANNEL_2},
};

void Set_PWM_Duty(PWM_Channel_t channel, uint16_t duty)
{
    if (channel >= PWM_CH_NUM)
    {
        return;
    }

    TIM_HandleTypeDef *htim = PWM_CHANNELS[channel].htim;
    uint32_t tim_channel = PWM_CHANNELS[channel].channel;

    __HAL_TIM_SET_COMPARE(htim, tim_channel, duty);
}
