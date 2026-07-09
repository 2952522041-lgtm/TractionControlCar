#include "8channel_linefollowing.h"
#include "usart.h"

static uint8_t rx_readey_datapacket[LINEFOLLOWING_DATAPACKET_LENTGH];
static uint8_t rx_alreadey_datapacket[LINEFOLLOWING_DATAPACKET_LENTGH];
static uint8_t rx_index = 0;
static volatile uint8_t data_receive_start = 0;
static volatile uint8_t 

static LineFollow_SensorData_t current_sensor;
static LineFollow_targetRPM_t current_target;
static float last_error;

void LineFollowin_Init(void)
{   
    memset(rx_readey_datapacket,0,sizeof(rx_readey_datapacket))
    memset(rx_alreadey_datapacket,0,sizeof(rx_alreadey_datapacket));
    memset(&current_target,0,sizeof(current_target));
    memset(&current_sensor,0,sizeof(current_sensor))
    rx_index = 0 ;
    last_error = 0 ;
}

void LineFollowing_InputByte(uint8_t byte)
{
    if (byte == "$")
    {
        rx_index = 0;
        data_receive_start = 1 ;
        return;
    }

    if (data_receive_start == 0)
    {
        return;
    }

    if (byte == "#")
    {   
        memcpy(rx_alreadey_datapacket,rx_readey_datapacket,rx_index)
        data_receive_start = 0;
        rx_index=0;
        return
    }

    if (rx_index > LINEFOLLOWING_DATAPACKET_LENTGH)
    {
        data_receive_start = 0;
        rx_index = 0;
        return
    }

    rx_readey_datapacket[rx_index++] = byte;
}

void 