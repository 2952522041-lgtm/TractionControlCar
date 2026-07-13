#include "app_task.h"
#include "encode.h"
#include "PWM.h"
#include "Motor.h"
#include "tb6612.h"
#include "8channel_linefollowing.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "dsp/controller_functions.h"

#include <string.h>

float SPEED_PID_KP = 0.0f;//还未调试
float SPEED_PID_KI = 0.0f;//还未调试
float SPEED_PID_KD = 0.0f;//还未调试
#define SPEED_PID_OUTPUT_LIMIT ((float)tb6612_max_rpm)
#define SPEED_SAMPLE_TASK_STACK_SIZE 256u
#define SPEED_SAMPLE_TASK_PRIORITY (tskIDLE_PRIORITY + 3u)
#define SPEED_PID_TASK_STACK_SIZE 256u
#define SPEED_PID_TASK_PRIORITY (tskIDLE_PRIORITY + 2u)
#define RECEIVE_TARGET_RPM_TASK_STACK_SIZE 256u
#define RECEIVE_TARGET_RPM_TASK_PRIORITY (tskIDLE_PRIORITY + 1u)
#define SPEED_SAMPLE_QUEUE_LENGTH 1u
#define SPEED_CONTROL_FREQUENCE 100.0f

static void speed_sample_task(void *pvParameters);
static void speed_pid_task(void *pvParameters);
static void receive_target_rpm_task(void *pvParameters);

static SemaphoreHandle_t speed_tick_sem = NULL;
static QueueHandle_t speed_sample_queue = NULL;
static arm_pid_instance_f32 speed_pid_left;
static arm_pid_instance_f32 speed_pid_right;
static float left_last_target_rpm = 0.0f;
static float right_last_target_rpm = 0.0f;

typedef struct
{
    float left_target_rpm;
    float right_target_rpm;

    float left_measured_rpm;
    float right_measured_rpm;
}speed_sample_t;

void APP_FREERTOS_Init(void)
{
    speed_tick_sem = xSemaphoreCreateBinary(); 
    speed_sample_queue = xQueueCreate(SPEED_SAMPLE_QUEUE_LENGTH,sizeof(speed_sample_t)); 

    speed_pid_left.Kp = SPEED_PID_KP;
    speed_pid_left.Ki = SPEED_PID_KI / SPEED_CONTROL_FREQUENCE; 
    speed_pid_left.Kd = SPEED_PID_KD * SPEED_CONTROL_FREQUENCE; 
    speed_pid_right.Kp = SPEED_PID_KP;
    speed_pid_right.Ki = SPEED_PID_KI / SPEED_CONTROL_FREQUENCE; 
    speed_pid_right.Kd = SPEED_PID_KD * SPEED_CONTROL_FREQUENCE; 
    arm_pid_init_f32(&speed_pid_left, 1);
    arm_pid_init_f32(&speed_pid_right, 1);

    xTaskCreate(speed_sample_task,
                "SpeedSampleTask",
                SPEED_SAMPLE_TASK_STACK_SIZE,
                NULL,
                SPEED_SAMPLE_TASK_PRIORITY,
                NULL);
    xTaskCreate(speed_pid_task,
                "SpeedPIDTask",
                SPEED_PID_TASK_STACK_SIZE,
                NULL,
                SPEED_PID_TASK_PRIORITY,
                NULL);
    xTaskCreate(receive_target_rpm_task,
                "ReceiveTargetRPMTask",
                RECEIVE_TARGET_RPM_TASK_STACK_SIZE,
                NULL,
                RECEIVE_TARGET_RPM_TASK_PRIORITY,
                NULL);
    
}

//100hz中断
void App_Timer100HZISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((speed_tick_sem != NULL) && (xTaskGetSchedulerState() ==taskSCHEDULER_RUNNING))
    {
        xSemaphoreGiveFromISR(speed_tick_sem, &xHigherPriorityTaskWoken);//获得信号量
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void APP_TIM3PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        App_Timer100HZISR();
    }

}

static void speed_sample_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(speed_tick_sem, portMAX_DELAY) == pdTRUE)
        {
            speed_sample_t sample;

            Encoder_Update(1.0f/SPEED_CONTROL_FREQUENCE);

            sample.left_target_rpm = Motor_GetTargetRPM(MOTOR_LEFT);
            sample.right_target_rpm = Motor_GetTargetRPM(MOTOR_RIGHT);

            sample.left_measured_rpm = Encoder_GetRPM(ENCODER_LEFT);
            sample.right_measured_rpm = Encoder_GetRPM(ENCODER_RIGHT);

            xQueueOverwrite(speed_sample_queue, &sample);
        }
    }
}

static void speed_pid_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        speed_sample_t sample;

        if (xQueueReceive(speed_sample_queue, &sample, portMAX_DELAY) == pdTRUE)
        {
           float left_error = sample.left_target_rpm - sample.left_measured_rpm;
           float right_error = sample.right_target_rpm - sample.right_measured_rpm;
           float left_output = 0;
           float right_output = 0;

           if (sample.left_target_rpm == 0.0f)
            {
                arm_pid_reset_f32(&speed_pid_left);
            }
            else
            {
                if (left_last_target_rpm * sample.left_target_rpm < 0.0f)
                {
                    arm_pid_reset_f32(&speed_pid_left);
                }

                float left_error = sample.left_target_rpm - sample.left_measured_rpm;
                left_output = arm_pid_f32(&speed_pid_left, left_error);
            }

            if (sample.right_target_rpm == 0.0f)
            {
                arm_pid_reset_f32(&speed_pid_right);
            }
            else
            {
                if (right_last_target_rpm * sample.right_target_rpm < 0.0f)
                {
                    arm_pid_reset_f32(&speed_pid_right);
                }

                float right_error = sample.right_target_rpm - sample.right_measured_rpm;

                right_output = arm_pid_f32(&speed_pid_right, right_error);
            }

  
            if (left_output > SPEED_PID_OUTPUT_LIMIT)
                left_output = SPEED_PID_OUTPUT_LIMIT;
            else if (left_output < -SPEED_PID_OUTPUT_LIMIT)
                left_output = -SPEED_PID_OUTPUT_LIMIT;

            if (right_output > SPEED_PID_OUTPUT_LIMIT)
                right_output = SPEED_PID_OUTPUT_LIMIT;
            else if (right_output < -SPEED_PID_OUTPUT_LIMIT)
                right_output = -SPEED_PID_OUTPUT_LIMIT;

            Motor_SetBothRPM(left_output, right_output);

            left_last_target_rpm = sample.left_target_rpm;
            right_last_target_rpm = sample.right_target_rpm;
        }
    }   
    
}

void receive_target_rpm_task(void *pvParameters)
{
    LineFollow_targetRPM_t target;
    (void)pvParameters;

    for (;;)
    {
        LineFollow_Update();

        if ((LineFollow_GetTargetRPM(&target) == 1u) &&
            (LineFollow_IsTargetTimeout(HAL_GetTick()) == 0u))
        {
            Motor_SetBothTargetRPM(target.left_target_rpm,target.right_target_rpm);
        }
        else
        {
            Motor_SetBothTargetRPM(0.0f, 0.0f);        
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void User_Init(void)
{
    Motor_Init();
    Encoder_Init();
    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, APP_TIM3PeriodElapsedCallback);
}