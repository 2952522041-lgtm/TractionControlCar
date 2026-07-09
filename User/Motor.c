#include "Motor.h"
#include "tb6612.h"

static float target_rpms[MOTOR_NUM]=
{
    [MOTOR_LEFT] = 0.0f,
    [MOTOR_RIGHT] = 0.0f,
};


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

void Motor_SetTargetRPM(Motor_ID_t motor, float rpm)
{
    if (motor >= MOTOR_NUM)
    {
        return;
    }

    if (rpm > tb6612_max_rpm)
    {
        rpm = tb6612_max_rpm;
    }
    if (rpm < -tb6612_max_rpm)
    {
        rpm = -tb6612_max_rpm;
    }
    target_rpms[motor] = rpm;

    if (rpm == 0.0f)
    {
        Motor_Stop(motor);
    }

}

void Motor_SetBothTargetRPM(float left_rpm, float right_rpm)
{
    Motor_SetTargetRPM(MOTOR_LEFT, left_rpm);
    Motor_SetTargetRPM(MOTOR_RIGHT, right_rpm);
}

float Motor_GetTargetRPM(Motor_ID_t motor)
{
   return target_rpms[motor];
}

void Motor_SetRPM(Motor_ID_t motor, float rpm)
{
    if (motor >= MOTOR_NUM)
    {
        return;
    }

    tb6612_SetRPM(motor_map[motor], rpm);
}

void Motor_SetBothRPM(float left_rpm, float right_rpm)
{
    Motor_SetRPM(MOTOR_LEFT, left_rpm);
    Motor_SetRPM(MOTOR_RIGHT, right_rpm);
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
