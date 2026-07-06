#include "motor.h"
#include "tb6612.h"

static const tb6612_Channel_t motor_map[MOTOR_NUM] =
{
    [MOTOR_LEFT] = tb6612_CH_LEFT,
    [MOTOR_RIGHT] = tb6612_CH_RIGHT,
};

static float Motor_LimitPercent(float percent)
{
    if (percent > 100.0f)
    {
        return 100.0f;
    }

    if (percent < -100.0f)
    {
        return -100.0f;
    }

    return percent;
}

void Motor_Init(void)
{
    tb6612_Init();
    Motor_StopAll();
}

void Motor_SetSpeedPercent(Motor_ID_t motor, float percent)
{
    float rpm;

    if (motor >= MOTOR_NUM)
    {
        return;
    }

    percent = Motor_LimitPercent(percent);
    rpm = percent / 100.0f * tb6612_max_rpm;

    tb6612_SetRPM(motor_map[motor], rpm);
}

void Motor_SetBothSpeedPercent(float left_percent, float right_percent)
{
    Motor_SetSpeedPercent(MOTOR_LEFT, left_percent);
    Motor_SetSpeedPercent(MOTOR_RIGHT, right_percent);
}

void Motor_Stop(Motor_ID_t motor)
{
    if (motor >= MOTOR_NUM)
    {
        return;
    }

    tb6612_Stop(motor_map[motor]);
}

void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_LEFT);
    Motor_Stop(MOTOR_RIGHT);
}

void Motor_Brake(Motor_ID_t motor)
{
    if (motor >= MOTOR_NUM)
    {
        return;
    }

    tb6612_Brake(motor_map[motor]);
}

void Motor_BrakeAll(void)
{
    Motor_Brake(MOTOR_LEFT);
    Motor_Brake(MOTOR_RIGHT);
}