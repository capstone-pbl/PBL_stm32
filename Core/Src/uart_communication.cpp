/*
 * uart_communication.cpp
 *
 *  Created on: Mar 22, 2026
 *      Author: zoon0
 */

#include "uart_communication.hpp"
#include <cstdio>
#include <cstring>
uart_comm *uart_comm::instance = nullptr;

uart_comm::uart_comm(UART_HandleTypeDef *handler) :
huart(handler),cx(0.0), cy(0.0), rx_byte(0), is_ready(false),idx(0)
{
	memset(rx_buffer,0,sizeof(rx_buffer));
	instance = this;
}

void uart_comm::begin_uart()
{

	HAL_UART_Receive_IT(this->huart, &rx_byte, 1);

}

void uart_comm::handle_rx()
{
	if(idx<63)rx_buffer[idx++]=rx_byte;
    if(rx_byte=='\n')
    {
      rx_buffer[idx-1]='\0';
      if(sscanf((char*)rx_buffer, "%f, %f",&cx,&cy)==2)
      {
       is_ready=true;
      }
      idx=0;
    }
	begin_uart();
}


float uart_comm::get_cx() const
{
	return this->cx;
}

float uart_comm::get_cy() const
{
	return this->cy;
}

bool uart_comm::get_isready()const
{
  return this->is_ready;

}

void uart_comm::reset_isready()
{
	this->is_ready=false;

}
extern unsigned volatile long cnt1;

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	cnt1++;
	if (huart->Instance == USART6)
	{
		if (uart_comm::instance != nullptr)
		{
               uart_comm::instance->handle_rx();
		}
	}
}


