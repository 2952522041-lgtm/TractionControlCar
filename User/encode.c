#include "encode.h"
#include "tim.h"

#define ENCODER_PPR 13.0f
#define ENCODER_QUAD_MULTIPLY 4.0f
#define MOTOR_GEAR_RATIO 30.0f

typedef struct
{
    TIM_HandleTypeDef *htim;
    int32_t last_count;
    float rpm;
} Encoder_Config_t;

static Encoder_Config_t encoders[ENCODER_NUM] =
{
    [ENCODER_LEFT] = {&htim2, 0, 0.0f},
    [ENCODER_RIGHT] = {&htim4, 0, 0.0f}
};

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    Encoder_ResetAll();
}

void Encoder_Reset(Encoder_ID_t encoder)
{
    if (encoder >= ENCODER_NUM)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(encoders[encoder].htim, 0);
    encoders[encoder].last_count = 0;
    encoders[encoder].rpm = 0.0f;
}

void Encoder_ResetAll(void)
{
    Encoder_Reset(ENCODER_LEFT);
    Encoder_Reset(ENCODER_RIGHT);
}

int32_t Encoder_GetCount(Encoder_ID_t encoder)
{
    if (encoder >= ENCODER_NUM)
    {
        return 0;
    }

    return (int32_t)__HAL_TIM_GET_COUNTER(encoders[encoder].htim);
}

void Encoder_Update(float dt_sec)
{
    if (dt_sec <= 0.0f)
    {
        return;
    }

    for (int i = 0; i < ENCODER_NUM; i++)
    {
        int32_t now = __HAL_TIM_GET_COUNTER(encoders[i].htim);
        int16_t diff = (int16_t)(now - encoders[i].last_count);

        encoders[i].last_count = now;

        float motor_rev = (float)diff / (ENCODER_PPR * ENCODER_QUAD_MULTIPLY);
        float wheel_rev = motor_rev / MOTOR_GEAR_RATIO;

        encoders[i].rpm = wheel_rev / dt_sec * 60.0f;
    }
}

float Encoder_GetRPM(Encoder_ID_t encoder)
{
    if (encoder >= ENCODER_NUM)
    {
        return 0.0f;
    }

    return encoders[encoder].rpm;
}
