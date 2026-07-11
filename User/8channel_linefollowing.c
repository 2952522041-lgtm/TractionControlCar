#include "8channel_linefollowing.h"

#include "usart.h"

#include <string.h>

/*
 * Yahboom digital frame:
 * $D,x1:1,x2:0,x3:1,x4:0,x5:1,x6:0,x7:1,x8:0#
 */
static uint8_t linefollow_digital_mode_command[] = "$0,0,1#";
static const float sensor_position[LF_SENSOR_NUM] =
{
    -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f
};

static uint8_t rx_building_packet[LINEFOLLOWING_DATAPACKET_LENGTH];
static uint8_t rx_pending_packet[LINEFOLLOWING_DATAPACKET_LENGTH];
static volatile uint16_t rx_building_length;
static volatile uint16_t rx_pending_length;
static volatile uint32_t rx_pending_tick_ms;
static volatile uint8_t rx_receiving;
static volatile uint8_t rx_packet_ready;
static volatile uint8_t uart_restart_required;
static uint8_t uart_rx_byte;
static uint8_t tracking_armed;

static LineFollow_SensorData_t current_sensor;
static LineFollow_targetRPM_t current_target;

static void LineFollow_UARTRxCompleteCallback(UART_HandleTypeDef *huart);
static void LineFollow_UARTErrorCallback(UART_HandleTypeDef *huart);
static void LineFollow_ServiceUART(void);
static void LineFollow_ExpireStaleData(void);
static uint8_t LineFollow_ParseDigitalFrame(const uint8_t *frame,
                                            uint16_t length,
                                            uint8_t values[LF_SENSOR_NUM]);
static void LineFollow_UpdateTarget(const uint8_t values[LF_SENSOR_NUM], uint32_t tick_ms);

static float LineFollow_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

void LineFollow_Init(void)
{
    memset(rx_building_packet, 0, sizeof(rx_building_packet));
    memset(rx_pending_packet, 0, sizeof(rx_pending_packet));
    memset(&current_sensor, 0, sizeof(current_sensor));
    memset(&current_target, 0, sizeof(current_target));

    rx_building_length = 0U;
    rx_pending_length = 0U;
    rx_pending_tick_ms = 0U;
    rx_receiving = 0U;
    rx_packet_ready = 0U;
    uart_restart_required = 0U;
    tracking_armed = 0U;

    if (HAL_UART_RegisterCallback(&huart2,
                                  HAL_UART_RX_COMPLETE_CB_ID,
                                  LineFollow_UARTRxCompleteCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UART_RegisterCallback(&huart2,
                                  HAL_UART_ERROR_CB_ID,
                                  LineFollow_UARTErrorCallback) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UART_Transmit(&huart2,
                          linefollow_digital_mode_command,
                          (uint16_t)(sizeof(linefollow_digital_mode_command) - 1U),
                          LINEFOLLOWING_UART_TIMEOUT_MS) != HAL_OK)
    {
        Error_Handler();
    }
}

void LineFollow_InputByte(uint8_t byte)
{
    if (byte == (uint8_t)'$')
    {
        rx_building_length = 0U;
        rx_building_packet[rx_building_length++] = byte;
        rx_receiving = 1U;
        return;
    }

    if (rx_receiving == 0U)
    {
        return;
    }

    if (rx_building_length >= LINEFOLLOWING_DATAPACKET_LENGTH)
    {
        rx_building_length = 0U;
        rx_receiving = 0U;
        return;
    }

    rx_building_packet[rx_building_length++] = byte;

    if (byte == (uint8_t)'#')
    {
        /* Keep the pending packet immutable until the task consumes it. */
        if (rx_packet_ready == 0U)
        {
            memcpy(rx_pending_packet, rx_building_packet, rx_building_length);
            rx_pending_length = rx_building_length;
            rx_pending_tick_ms = HAL_GetTick();
            __DMB();
            rx_packet_ready = 1U;
        }

        rx_building_length = 0U;
        rx_receiving = 0U;
    }
}

void LineFollow_Processdata(void)
{
    uint8_t frame[LINEFOLLOWING_DATAPACKET_LENGTH];
    uint8_t values[LF_SENSOR_NUM];
    uint16_t frame_length;
    uint32_t frame_tick_ms;

    LineFollow_ServiceUART();

    if (rx_packet_ready == 0U)
    {
        return;
    }

    frame_length = rx_pending_length;
    frame_tick_ms = rx_pending_tick_ms;
    if ((frame_length == 0U) || (frame_length > sizeof(frame)))
    {
        rx_packet_ready = 0U;
        return;
    }

    memcpy(frame, rx_pending_packet, frame_length);
    __DMB();
    rx_packet_ready = 0U;

    if ((uint32_t)(HAL_GetTick() - frame_tick_ms) > LINEFOLLOWING_TIMEOUT_MS)
    {
        return;
    }

    if (LineFollow_ParseDigitalFrame(frame, frame_length, values) == 0U)
    {
        return;
    }

    memcpy(current_sensor.value, values, sizeof(values));
    current_sensor.tick_ms = frame_tick_ms;
    current_sensor.valid = 1U;

    LineFollow_UpdateTarget(values, frame_tick_ms);
}

void LineFollow_GetTargetRPM(LineFollow_targetRPM_t *target)
{
    if (target == NULL)
    {
        return;
    }

    LineFollow_ExpireStaleData();
    *target = current_target;
}

void LineFollow_GetSensorData(LineFollow_SensorData_t *data)
{
    if (data == NULL)
    {
        return;
    }

    LineFollow_ExpireStaleData();
    *data = current_sensor;
}

static void LineFollow_UARTRxCompleteCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    LineFollow_InputByte(uart_rx_byte);
    if (HAL_UART_Receive_IT(huart, &uart_rx_byte, 1U) != HAL_OK)
    {
        uart_restart_required = 1U;
    }
    else
    {
        uart_restart_required = 0U;
    }
}

static void LineFollow_UARTErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    if (huart->RxState == HAL_UART_STATE_READY)
    {
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        if (HAL_UART_Receive_IT(huart, &uart_rx_byte, 1U) != HAL_OK)
        {
            uart_restart_required = 1U;
        }
        else
        {
            uart_restart_required = 0U;
        }
    }
}

static void LineFollow_ServiceUART(void)
{
    if ((uart_restart_required != 0U) &&
        (huart2.RxState == HAL_UART_STATE_READY))
    {
        if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U) == HAL_OK)
        {
            uart_restart_required = 0U;
        }
    }
}

static void LineFollow_ExpireStaleData(void)
{
    if ((current_sensor.valid != 0U) &&
        ((uint32_t)(HAL_GetTick() - current_sensor.tick_ms) > LINEFOLLOWING_TIMEOUT_MS))
    {
        current_sensor.valid = 0U;
        current_target.left_target_rpm = 0.0f;
        current_target.right_target_rpm = 0.0f;
        current_target.valid = 0U;
        tracking_armed = 0U;
    }
}

static uint8_t LineFollow_ParseDigitalFrame(const uint8_t *frame,
                                            uint16_t length,
                                            uint8_t values[LF_SENSOR_NUM])
{
    uint16_t position = 0U;

    if ((frame == NULL) || (values == NULL) || (length < 3U))
    {
        return 0U;
    }

    if ((frame[position++] != (uint8_t)'$') ||
        (frame[position++] != (uint8_t)'D'))
    {
        return 0U;
    }

    for (uint8_t channel = 0U; channel < LF_SENSOR_NUM; channel++)
    {
        if ((position >= length) || (frame[position++] != (uint8_t)','))
        {
            return 0U;
        }

        if ((position >= length) ||
            ((frame[position] != (uint8_t)'x') && (frame[position] != (uint8_t)'X')))
        {
            return 0U;
        }
        position++;

        if ((position >= length) ||
            (frame[position++] != (uint8_t)((uint8_t)'1' + channel)))
        {
            return 0U;
        }

        if ((position >= length) || (frame[position++] != (uint8_t)':'))
        {
            return 0U;
        }

        if ((position >= length) ||
            ((frame[position] != (uint8_t)'0') && (frame[position] != (uint8_t)'1')))
        {
            return 0U;
        }

        values[channel] = (uint8_t)(frame[position++] - (uint8_t)'0');
    }

    if ((position >= length) || (frame[position++] != (uint8_t)'#'))
    {
        return 0U;
    }

    return (position == length) ? 1U : 0U;
}

static void LineFollow_UpdateTarget(const uint8_t values[LF_SENSOR_NUM], uint32_t tick_ms)
{
    float weighted_position = 0.0f;
    uint8_t active_count = 0U;

    for (uint8_t channel = 0U; channel < LF_SENSOR_NUM; channel++)
    {
        if (values[channel] == LINEFOLLOWING_ACTIVE_VALUE)
        {
            weighted_position += sensor_position[channel];
            active_count++;
        }
    }

    current_target.tick_ms = tick_ms;

    if (active_count == 0U)
    {
        current_target.left_target_rpm = 0.0f;
        current_target.right_target_rpm = 0.0f;
        current_target.valid = 0U;
        return;
    }

    /* Do not move on a uniform power-up/fault pattern. */
    if (tracking_armed == 0U)
    {
        if (active_count == LF_SENSOR_NUM)
        {
            current_target.left_target_rpm = 0.0f;
            current_target.right_target_rpm = 0.0f;
            current_target.valid = 0U;
            return;
        }

        tracking_armed = 1U;
    }

    {
        float line_error = weighted_position / (float)active_count;
        float turn_rpm = line_error * LINEFOLLOWING_TURN_PARAMETER;

        /* Do not reverse either wheel during normal line following. */
        turn_rpm = LineFollow_Clamp(turn_rpm,
                                    -LINEFOLLOWING_BASE_RPM,
                                    LINEFOLLOWING_BASE_RPM);

        current_target.left_target_rpm = LINEFOLLOWING_BASE_RPM + turn_rpm;
        current_target.right_target_rpm = LINEFOLLOWING_BASE_RPM - turn_rpm;
        current_target.valid = 1U;
    }
}
