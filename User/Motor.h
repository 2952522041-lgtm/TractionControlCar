#pragma once

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_NUM
} Motor_ID_t;

void Motor_Init(void);

void Motor_SetRPM(Motor_ID_t motor, float rpm);
void Motor_SetBothRPM(float left_rpm, float right_rpm);

void Motor_SetTargetRPM(Motor_ID_t motor, float rpm);
void Motor_SetBothTargetRPM(float left_rpm, float right_rpm);
float Motor_GetTargetRPM(Motor_ID_t motor);

void Motor_SetSpeedPercent(Motor_ID_t motor, float percent);
void Motor_SetBothSpeedPercent(float left_percent, float right_percent);

void Motor_Stop(Motor_ID_t motor);
void Motor_StopAll(void);

void Motor_Brake(Motor_ID_t motor);
void Motor_BrakeAll(void);
