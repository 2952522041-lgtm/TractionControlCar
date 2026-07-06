#include "car.h"
#include "motor.h"

static float Car_LimitPercent(float value)
{
    if (value > 100.0f)
    {
        return 100.0f;
    }

    if (value < -100.0f)
    {
        return -100.0f;
    }

    return value;
}

void Car_Init(void)
{
    Motor_Init();
}

void Car_Drive(float throttle, float steer)
{
    float left_speed;
    float right_speed;

    throttle = Car_LimitPercent(throttle);
    steer = Car_LimitPercent(steer);

    left_speed = throttle + steer;
    right_speed = throttle - steer;

    left_speed = Car_LimitPercent(left_speed);
    right_speed = Car_LimitPercent(right_speed);

    Motor_SetBothSpeedPercent(left_speed, right_speed);
}

void Car_Stop(void)
{
    Motor_StopAll();
}

void Car_Brake(void)
{
    Motor_BrakeAll();
}