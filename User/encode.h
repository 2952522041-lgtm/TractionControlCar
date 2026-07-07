#pragma once

#include "main.h"

typedef enum
{
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_NUM
} Encoder_ID_t;

void Encoder_Init(void);
void Encoder_Reset(Encoder_ID_t encoder);
void Encoder_ResetAll(void);
int32_t Encoder_GetCount(Encoder_ID_t encoder);
void Encoder_Update(float dt_sec);
float Encoder_GetRPM(Encoder_ID_t encoder);
