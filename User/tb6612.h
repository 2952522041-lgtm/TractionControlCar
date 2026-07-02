#pragma once
#include "main.h"

#define tb6612_PWM_MAX_DUTY 3599u
#define tb6612_rpm_direction 1.0f
#define tb6612_max_rpm 100.0f

typedef enum
{
    tb6612_DIR_STOP = 0,
    tb6612_DIR_FORWARD,
    tb6612_DIR_BACKWARD,
    tb6612_DIR_BRAKE
}tb6612_Direction_t;

typedef enum
{
    tb6612_CH_LEFT = 0,
    tb6612_CH_RIGHT,
    tb6612_CH_NUM,

}tb6612_Channel_t;

typedef struct
{
    GPIO_TypeDef *IN1_GPIO_Port;
    uint16_t IN1_Pin;
    GPIO_TypeDef *IN2_GPIO_Port;
    uint16_t IN2_Pin;
    uint32_t pwm_channel;
    float speed;
} tb6612_Config_t;

static tb6612_Config_t tb6612s[tb6612_CH_NUM] = 
{
    [tb6612_CH_LEFT] = {AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, PWM_CH_LEFT, 0.0f},
    [tb6612_CH_RIGHT] = {BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, PWM_CH_RIGHT, 0.0f}
};