#include "app_task.h"

#include "8channel_linefollowing.h"
#include "Motor.h"
#include "encode.h"
#include "tb6612.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "dsp/controller_functions.h"

/* Keep these at zero until the speed loop is calibrated on the real car. */
#define SPEED_PID_KP 0.0f
#define SPEED_PID_KI 0.0f
#define SPEED_PID_KD 0.0f
#define SPEED_PID_OUTPUT_LIMIT tb6612_max_rpm

#define SPEED_CONTROL_FREQUENCY_HZ 100.0f
#define SPEED_CONTROL_PERIOD_SEC (1.0f / SPEED_CONTROL_FREQUENCY_HZ)
#define SPEED_SAMPLE_QUEUE_LENGTH 1U

/* CubeMX maps its default CMSIS task to FreeRTOS priority 24. */
#define SPEED_SAMPLE_TASK_STACK_SIZE 160U
#define SPEED_SAMPLE_TASK_PRIORITY (tskIDLE_PRIORITY + 27U)
#define SPEED_PID_TASK_STACK_SIZE 160U
#define SPEED_PID_TASK_PRIORITY (tskIDLE_PRIORITY + 26U)
#define LINE_FOLLOW_TASK_STACK_SIZE 192U
#define LINE_FOLLOW_TASK_PRIORITY (tskIDLE_PRIORITY + 25U)
#define LINE_FOLLOW_TASK_PERIOD_MS 10U
#define LINE_FOLLOW_SENSOR_STARTUP_MS 1000U

typedef struct
{
    float left_target_rpm;
    float right_target_rpm;
    float left_measured_rpm;
    float right_measured_rpm;
} speed_sample_t;

typedef struct
{
    float left_rpm;
    float right_rpm;
} speed_target_t;

static void speed_sample_task(void *pvParameters);
static void speed_pid_task(void *pvParameters);
static void line_follow_task(void *pvParameters);
static void APP_TIM3PeriodElapsedCallback(TIM_HandleTypeDef *htim);
static float speed_pid_calculate(arm_pid_instance_f32 *pid,
                                 float target_rpm,
                                 float measured_rpm,
                                 float *last_target_rpm);
static void speed_target_publish(float left_rpm, float right_rpm);
static void speed_target_read(float *left_rpm, float *right_rpm);

static SemaphoreHandle_t speed_tick_sem;
static StaticSemaphore_t speed_tick_sem_buffer;

static QueueHandle_t speed_sample_queue;
static StaticQueue_t speed_sample_queue_buffer;
static uint8_t speed_sample_queue_storage[sizeof(speed_sample_t) * SPEED_SAMPLE_QUEUE_LENGTH];

static arm_pid_instance_f32 left_speed_pid;
static arm_pid_instance_f32 right_speed_pid;
static float left_last_target_rpm;
static float right_last_target_rpm;
static speed_target_t requested_speed;

static StaticTask_t speed_sample_task_buffer;
static StackType_t speed_sample_task_stack[SPEED_SAMPLE_TASK_STACK_SIZE];
static StaticTask_t speed_pid_task_buffer;
static StackType_t speed_pid_task_stack[SPEED_PID_TASK_STACK_SIZE];
static StaticTask_t line_follow_task_buffer;
static StackType_t line_follow_task_stack[LINE_FOLLOW_TASK_STACK_SIZE];

void APP_FREERTOS_Init(void)
{
    TaskHandle_t speed_sample_handle;
    TaskHandle_t speed_pid_handle;
    TaskHandle_t line_follow_handle;

    speed_tick_sem = xSemaphoreCreateBinaryStatic(&speed_tick_sem_buffer);
    speed_sample_queue = xQueueCreateStatic(SPEED_SAMPLE_QUEUE_LENGTH,
                                            sizeof(speed_sample_t),
                                            speed_sample_queue_storage,
                                            &speed_sample_queue_buffer);

    if ((speed_tick_sem == NULL) || (speed_sample_queue == NULL))
    {
        Error_Handler();
    }

    left_speed_pid.Kp = SPEED_PID_KP;
    left_speed_pid.Ki = SPEED_PID_KI / SPEED_CONTROL_FREQUENCY_HZ;
    left_speed_pid.Kd = SPEED_PID_KD * SPEED_CONTROL_FREQUENCY_HZ;
    arm_pid_init_f32(&left_speed_pid, 1);

    right_speed_pid.Kp = SPEED_PID_KP;
    right_speed_pid.Ki = SPEED_PID_KI / SPEED_CONTROL_FREQUENCY_HZ;
    right_speed_pid.Kd = SPEED_PID_KD * SPEED_CONTROL_FREQUENCY_HZ;
    arm_pid_init_f32(&right_speed_pid, 1);

    speed_sample_handle = xTaskCreateStatic(speed_sample_task,
                                            "SpeedSample",
                                            SPEED_SAMPLE_TASK_STACK_SIZE,
                                            NULL,
                                            SPEED_SAMPLE_TASK_PRIORITY,
                                            speed_sample_task_stack,
                                            &speed_sample_task_buffer);
    speed_pid_handle = xTaskCreateStatic(speed_pid_task,
                                         "SpeedPID",
                                         SPEED_PID_TASK_STACK_SIZE,
                                         NULL,
                                         SPEED_PID_TASK_PRIORITY,
                                         speed_pid_task_stack,
                                         &speed_pid_task_buffer);
    line_follow_handle = xTaskCreateStatic(line_follow_task,
                                           "LineFollow",
                                           LINE_FOLLOW_TASK_STACK_SIZE,
                                           NULL,
                                           LINE_FOLLOW_TASK_PRIORITY,
                                           line_follow_task_stack,
                                           &line_follow_task_buffer);

    if ((speed_sample_handle == NULL) ||
        (speed_pid_handle == NULL) ||
        (line_follow_handle == NULL))
    {
        Error_Handler();
    }
}

void App_Timer100HZISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((speed_tick_sem != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        xSemaphoreGiveFromISR(speed_tick_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void User_Init(void)
{
    Motor_Init();
    Encoder_Init();

    if (HAL_TIM_RegisterCallback(&htim3,
                                 HAL_TIM_PERIOD_ELAPSED_CB_ID,
                                 APP_TIM3PeriodElapsedCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
        Error_Handler();
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
    TickType_t last_sample_tick = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(speed_tick_sem, portMAX_DELAY) == pdTRUE)
        {
            speed_sample_t sample;
            TickType_t current_tick = xTaskGetTickCount();
            TickType_t elapsed_ticks = current_tick - last_sample_tick;
            float elapsed_sec = SPEED_CONTROL_PERIOD_SEC;

            last_sample_tick = current_tick;
            if (elapsed_ticks != 0U)
            {
                elapsed_sec = (float)elapsed_ticks / (float)configTICK_RATE_HZ;
            }

            Encoder_Update(elapsed_sec);

            speed_target_read(&sample.left_target_rpm, &sample.right_target_rpm);
            sample.left_measured_rpm = Encoder_GetRPM(ENCODER_LEFT);
            sample.right_measured_rpm = Encoder_GetRPM(ENCODER_RIGHT);

            (void)xQueueOverwrite(speed_sample_queue, &sample);
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
            float left_output = speed_pid_calculate(&left_speed_pid,
                                                    sample.left_target_rpm,
                                                    sample.left_measured_rpm,
                                                    &left_last_target_rpm);
            float right_output = speed_pid_calculate(&right_speed_pid,
                                                     sample.right_target_rpm,
                                                     sample.right_measured_rpm,
                                                     &right_last_target_rpm);

            Motor_SetBothRPM(left_output, right_output);
        }
    }
}

static void line_follow_task(void *pvParameters)
{
    TickType_t last_wake_time;

    (void)pvParameters;

    speed_target_publish(0.0f, 0.0f);

    /* The sensor needs time after power-up before accepting the mode command. */
    vTaskDelay(pdMS_TO_TICKS(LINE_FOLLOW_SENSOR_STARTUP_MS));
    LineFollow_Init();
    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        LineFollow_targetRPM_t target;

        LineFollow_Processdata();
        LineFollow_GetTargetRPM(&target);

        if (target.valid != 0U)
        {
            speed_target_publish(target.left_target_rpm, target.right_target_rpm);
        }
        else
        {
            speed_target_publish(0.0f, 0.0f);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LINE_FOLLOW_TASK_PERIOD_MS));
    }
}

static float speed_pid_calculate(arm_pid_instance_f32 *pid,
                                 float target_rpm,
                                 float measured_rpm,
                                 float *last_target_rpm)
{
    float output;

    if (target_rpm == 0.0f)
    {
        arm_pid_reset_f32(pid);
        *last_target_rpm = 0.0f;
        return 0.0f;
    }

    if (((*last_target_rpm >= 0.0f) && (target_rpm < 0.0f)) ||
        ((*last_target_rpm <= 0.0f) && (target_rpm > 0.0f)))
    {
        arm_pid_reset_f32(pid);
    }

    /* Target RPM is the feed-forward term; the PID adds encoder correction. */
    output = target_rpm + arm_pid_f32(pid, target_rpm - measured_rpm);

    if (output > SPEED_PID_OUTPUT_LIMIT)
    {
        output = SPEED_PID_OUTPUT_LIMIT;
    }
    else if (output < -SPEED_PID_OUTPUT_LIMIT)
    {
        output = -SPEED_PID_OUTPUT_LIMIT;
    }

    *last_target_rpm = target_rpm;
    return output;
}

static void speed_target_publish(float left_rpm, float right_rpm)
{
    taskENTER_CRITICAL();
    requested_speed.left_rpm = left_rpm;
    requested_speed.right_rpm = right_rpm;
    taskEXIT_CRITICAL();
}

static void speed_target_read(float *left_rpm, float *right_rpm)
{
    taskENTER_CRITICAL();
    *left_rpm = requested_speed.left_rpm;
    *right_rpm = requested_speed.right_rpm;
    taskEXIT_CRITICAL();
}
