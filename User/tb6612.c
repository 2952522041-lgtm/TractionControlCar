#include "tb6612.h"
#include "PWM.h"
#include "tim.h"

void tb6612_Init(void);
void tb6612_SetDirection(tb6612_Channel_t tb6612_Channel, tb6612_Direction_t direction);
void tb6612_Stop(void);
void tb6612_Brake(void);
void tb6612_SetRPM(float rpm);

void tb6612_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    tb6612_Stop(tb6612_CH_LEFT);
    tb6612_Stop(tb6612_CH_RIGHT);
}

void tb6612_SetDuty(tb6612_Channel_t tb6612_Channel, uint32_t duty)
{
    if (duty > tb6612_PWM_MAX_DUTY)
    {
       duty = tb6612_PWM_MAX_DUTY;
    }

    channel = tb6612s[tb6612_Channel].pwm_channel;

    Set_PWM_Duty(PWM_Channel_t channel, duty)
}

void tb6612_SetDirection(tb6612_Channel_t tb6612_Channel, tb6612_Direction_t direction)
{
    GPIO_TypeDef *IN1_GPIO_Port = tb6612s[tb6612_Channel].IN1_GPIO_Port;
    uint16_t IN1_Pin = tb6612s[tb6612_Channel].IN1_Pin;
    GPIO_TypeDef *IN2_GPIO_Port = tb6612s[tb6612_Channel].IN2_GPIO_Port;
    uint16_t IN2_Pin = tb6612s[tb6612_Channel].IN2_Pin;

    switch (direction)
    {
        
        case tb6612_DIR_FORWARD:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            break;
        case tb6612_DIR_BACKWARD:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            break;
        case tb6612_DIR_BRAKE:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            break;
        case tb6612_DIR_STOP:
        default:
            HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            break;    
    }
}

void tb6612_Stop(tb6612_Channel_t tb6612_Channel)
{
    tb6612_SetDuty(tb6612_Channel, 0);
    tb6612_SetDirection(tb6612_Channel, tb6612_DIR_STOP);
}

void tb6612_Brake(tb6612_Channel_t tb6612_Channel)
{
    tb6612_SetDuty(tb6612_Channel, 0);
    tb6612_SetDirection(tb6612_Channel, tb6612_DIR_BRAKE);
}

float abs(float number)
{
    if (number < 0)
    {
        return -number;
    }
    else
    {
        return number;
    }
}

void tb6612_SetRPM(tb6612_Channel_t tb6612_Channel, float rpm)
{
    float motor_rpm = rpm * tb6612_rpm_direction;
    uint32_t duty = 0;

    if (motor_rpm == 0.0f)
    {
        tb6612_Stop(tb6612_Channel);
        return;
    }
    if (motor_rpm > tb6612_max_rpm)
    {
        motor_rpm = tb6612_max_rpm;
    }
    else if (motor_rpm < -tb6612_max_rpm)
    {
        motor_rpm = -tb6612_max_rpm;
    }
    duty = (uint32_t)(abs(motor_rpm) / tb6612_max_rpm) * (float)tb6612_PWM_MAX_DUTY;

    if (motor_rpm > 0.0f)
    {
        tb6612_SetDirection(tb6612_Channel, tb6612_DIR_FORWARD);
    }
    else
    {
        tb6612_SetDirection(tb6612_Channel, tb6612_DIR_BACKWARD);
    }

    tb6612_SetDuty(tb6612_Channel, duty);

}