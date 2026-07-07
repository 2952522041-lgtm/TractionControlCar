#include "Car.h"
#include "Motor.h"

void Car_Init(void)
{
    Motor_Init();
}

void Car_Drive(float throttle_rpm, float steer_rpm)
{
    float left_rpm;
    float right_rpm;

    left_rpm = throttle_rpm + steer_rpm;
    right_rpm = throttle_rpm - steer_rpm;

    Motor_SetBothRPM(left_rpm, right_rpm);
}

void Car_Stop(void)
{
    Motor_StopAll();
}

void Car_Brake(void)
{
    Motor_BrakeAll();
}
