#include "8channel_linefollowing.h"
#include "usart.h"
#include <string.h>

const float weight[LF_SENSOR_NUM]=
{
    -7.0f, -5.0f, -3.0f, -1.0f,
     1.0f,  3.0f,  5.0f,  7.0f
};

static uint8_t uart_rx_byte;
static uint8_t rx_readey_datapacket[LINEFOLLOWING_DATAPACKET_LENTGH];
static uint8_t rx_alreadey_datapacket[LINEFOLLOWING_DATAPACKET_LENTGH];
static uint8_t rx_index = 0;//目前接受到第几位
static volatile uint8_t data_receive_start = 0;//是否已经接受到$,正在接受包
static volatile uint8_t data_alread_receive_len=0;//完整接受到的数据包长度
static volatile uint8_t data_alread_receive=0;//是否有新数据


static LineFollow_SensorData_t current_sensor;
static LineFollow_targetRPM_t current_target;
static float last_error;

void LineFollow_Init(void)
{   
    memset(rx_readey_datapacket,0,sizeof(rx_readey_datapacket));
    memset(rx_alreadey_datapacket,0,sizeof(rx_alreadey_datapacket));
    memset(&current_target,0,sizeof(current_target));
    memset(&current_sensor,0,sizeof(current_sensor));
    rx_index = 0 ;
    data_alread_receive_len=0;
    data_receive_start=0;
    data_alread_receive=0;
    last_error = 0 ;
}
//开始接受信号
void LineFollow_StartUartReceive()
{
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte,1);
}

//每个中断时该做什么
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    LineFollow_UartRxCpltCallback(huart);             
}

//回调函数
void LineFollow_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        LineFollow_InputByte(uart_rx_byte);
        HAL_UART_Receive_IT(&huart2,&uart_rx_byte,1);
    }
}

//传入的字节做初始化处理，全都放进一个数组
void LineFollow_InputByte(uint8_t byte)
{
    if (byte == '$')
    {
        rx_index = 0;
        data_receive_start = 1 ;
        return;
    }

    if (data_receive_start == 0)
    {
        return;
    }

    if (byte == '#')
    {   
        memcpy(rx_alreadey_datapacket,rx_readey_datapacket,rx_index);
        data_receive_start = 0;
        data_alread_receive_len=rx_index;
        data_alread_receive=1u;
        rx_index=0;
        return;
    }

    if (rx_index >= LINEFOLLOWING_DATAPACKET_LENTGH)
    {
        data_receive_start = 0;
        rx_index = 0;
        return;
    }

    rx_readey_datapacket[rx_index++] = byte;
}

//将传入的数据处理，保存为相应传感器的值为八位数组
static uint8_t LineFollow_Sensordata(uint8_t *data, uint8_t len, uint8_t sensor_out[LF_SENSOR_NUM])
{
    uint8_t count = 0;
    uint8_t i = 0;
    
    for (i=0; i+1u < len; i ++)
    {
        if (data[i] == ':')
        {   
            if ((data[i+1u] == '0') || data[i+1u] == '1')
            {
                if(count<LF_SENSOR_NUM)
                {
                    sensor_out[count] = data[i+1u] - '0';
                    count++;
                }
            }
        }
    }
    if (count == LF_SENSOR_NUM)
    {
        return 1u;
    }
    return 0u;
}

//计算偏差加权和
static float LF_Error(uint8_t sensordata[LF_SENSOR_NUM], uint8_t *has_line)
{
    float sum = 0;
    float count = 0;
    uint8_t i;
    *has_line=0u;//判断是否丢线
    
    for (i = 0; i < LF_SENSOR_NUM; i++)
    {
        if (sensordata[i] == 0u)
        {
            sum += weight[i];
            count += 1;
        }
    } 

    if (count == 0.0f)
    {
        return last_error;
    }

    *has_line = 1u;
    last_error = sum / count;
    return last_error;
}

//对rpm进行限制
static float LF_RPMLimit(float value,float min,float max)
{
    if (value > max)                              
    {
        return max;                               
    }

    if (value < min)                              
    {
        return min;                               
    }

    return value; 
}

//传入传感器数值，更新目标速度结构体
static void LF_UpdataTargetRPMformSensor(uint8_t sensor[LF_SENSOR_NUM])
{
    uint8_t has_line;
    float error;
    float turn;
    LineFollow_targetRPM_t target;
    uint32_t now_ms = HAL_GetTick();

    error = LF_Error(sensor, &has_line);

    if (has_line == 0u)//丢线时停止
    {
        target.left_target_rpm = 0.0f;
        target.right_target_rpm = 0.0f;
        target.tick_ms = now_ms;
        target.valid = 0u;
    }
    else
    {
        turn = error * LINEFOLLOWING_TURN_PARAMETER;
        target.left_target_rpm = LF_CarBaseRPM + turn;
        target.right_target_rpm = LF_CarBaseRPM - turn;
        target.left_target_rpm = LF_RPMLimit(target.left_target_rpm,LF_CarMINRPM,LF_CarMAXRPM);
        target.right_target_rpm = LF_RPMLimit(target.right_target_rpm,LF_CarMINRPM,LF_CarMAXRPM);
        target.tick_ms = now_ms;
        target.valid = 1u;

    }
    current_target = target;
}

//更新所有值
void LineFollow_Update()
{   
    uint8_t data_copy[LINEFOLLOWING_DATAPACKET_LENTGH];
    uint8_t data_len_copy;
    uint8_t sensor_temp[LF_SENSOR_NUM];
    LineFollow_SensorData_t sensor_new;

    if (data_alread_receive == 0u)
    {
        return;
    }

    data_len_copy = data_alread_receive_len;
    memcpy(data_copy, rx_alreadey_datapacket,data_len_copy);
    data_alread_receive = 0u;

    if ( LineFollow_Sensordata(data_copy, data_len_copy, sensor_temp) == 0u)
    {
        return;
    }

    memcpy(sensor_new.value,sensor_temp,sizeof(sensor_new.value));
    sensor_new.tick_ms=HAL_GetTick();
    sensor_new.valid = 1u;

    current_sensor = sensor_new;
    LF_UpdataTargetRPMformSensor(sensor_temp);
}

//外部获取传感器值
uint8_t LineFollow_GetSensorData(LineFollow_SensorData_t *sensor)
{
    if (sensor == 0)
    {
        return 0u;
    }

    *sensor = current_sensor;
    return sensor->valid;
}

//外部获取目标速度值
uint8_t LineFollow_GetTargetRPM(LineFollow_targetRPM_t *target)
{
                            
    if (target == 0)                                   
    {
        return 0u;                                     
    }
             
    *target = current_target;                           
            
    return target->valid;                              
}

//判断数值是否超时失效
uint8_t LineFollow_IsTargetTimeout(uint32_t now_ms)
{
    uint8_t valid;
    uint32_t tick_ms;
    
    valid = current_target.valid;
    tick_ms = current_target.tick_ms;

    if (valid == 0u)
    {
        return 1u;
    }

    if ((now_ms - tick_ms) > LINEFOLLOWING_TIMEOUT_MS)
    {
        return 1u;
    }

    return 0u;
}