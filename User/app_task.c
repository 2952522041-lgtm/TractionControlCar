#include "app_task.h"
#include "encode.h"
#include "PWM.h"
#include "Motor.h"
#include "tb6612.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "dsp/controller_functions.h"

#include <string.h>

#define SPEED_PID_KP 0.0f
#define SPEED_PID_KI 0.0f
#define SPEED_PID_KD 0.0f
#define SPEED_PID_OUTPUT_LIMIT ((float)tb6612_PWM_MAX_DUTY)
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

static SemaphoreHandle_t speed_tick_sem = NULL;
static QueueHandle_t speed_sample_queue = NULL;
static arm_pid_instance_f32 speed_pid;
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
    speed_tick_sem = xSemaphoreCreateBinary(); //create a semaphore for speed tick
    speed_sample_queue = xQueueCreate(SPEED_SAMPLE_QUEUE_LENGTH,sizeof(speed_sample_t)); //create a queue for speed sample

    speed_pid.Kp = SPEED_PID_KP;
    speed_pid.Ki = SPEED_PID_KI / SPEED_CONTROL_FREQUENCE; //Ki is divided by the control frequency to get the integral gain per sample;
    speed_pid.Kd = SPEED_PID_KD * SPEED_CONTROL_FREQUENCE; //Kd is multiplied by the control frequency to get the derivative gain per sample;
    arm_pid_init_f32(&speed_pid, 1); //initialize the PID controller

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
void App_Timer100HZISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((speed_tick_sem != NULL) && (xTaskGetSchedulerState() ==taskSCHEDULER_RUNNING))
    {
        xSemaphoreGiveFromISR(speed_tick_sem, &xHigherPriorityTaskWoken);
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

            sample.left_target_rpm = Motor_GetTargetRPM(MOTOR_LEFT);
            sample.right_target_rpm = Motor_GetTargetRPM(MOTOR_RIGHT);

            sample.left_measured_rpm = encoder_GetRPM(ENCODER_LEFT);
            sample.right_measured_rpm = encoder_GetRPM(ENCODER_RIGHT);

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
           float left_output;
           float right_output;

           if (sample.left_target_rpm == 0.0f)
           {
                arm_pid_reset_f32(&speed_pid);
                left_last_target_rpm = 0.0f;
                Motor_SetRPM(MOTOR_LEFT,0.0f);
                continue;
           }

           if (sample.right_target_rpm == 0.0f)
           {
                arm_pid_reset_f32(&speed_pid);
                right_last_target_rpm = 0.0f;
                Motor_SetRPM(MOTOR_RIGHT,0.0f);
                continue;
           }

           if((left_last_target_rpm >= 0.0f && sample.left_target_rpm < 0.0f) ||
               (left_last_target_rpm <= 0.0f && sample.left_target_rpm > 0.0f))
            {
                arm_pid_reset_f32(&speed_pid);
            } 

            if ((right_last_target_rpm >= 0.0f && sample.right_target_rpm < 0.0f) ||
               (right_last_target_rpm <= 0.0f && sample.right_target_rpm > 0.0f))
            {
                arm_pid_reset_f32(&speed_pid);
            }

           left_output = arm_pid_f32(&speed_pid, left_error);
           left_output = (left_output > SPEED_PID_OUTPUT_LIMIT) ? SPEED_PID_OUTPUT_LIMIT : left_output;
           right_output = arm_pid_f32(&speed_pid, right_error);
           right_output = (right_output > SPEED_PID_OUTPUT_LIMIT) ? SPEED_PID_OUTPUT_LIMIT : right_output;
            Motor_SetBothRPM(left_output, right_output);
           left_last_target_rpm = sample.left_target_rpm;
           right_last_target_rpm = sample.right_target_rpm;

        }
    }
}

void receive_target_rpm_task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        // Wait for new target RPM values to be received
        // This is a placeholder for the actual implementation of receiving target RPM values
        // You can replace this with your own implementation, such as reading from a UART or CAN bus

        float left_target_rpm = 0.0f; // Replace with actual received value
        float right_target_rpm = 0.0f; // Replace with actual received value

        Motor_SetBothTargetRPM(left_target_rpm, right_target_rpm);

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as needed
    }
}


void User_Init(void)
{
    Motor_Init();
    encoder_Init();
    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, APP_TIM3PeriodElapsedCallback);
}