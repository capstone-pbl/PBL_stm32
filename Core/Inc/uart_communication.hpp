/*
 * uart_communication.hpp
 *
 *  Created on: Apr 12, 2026
 *      Author: zoon0
 */

#ifndef INC_UART_COMMUNICATION_HPP_
#define INC_UART_COMMUNICATION_HPP_

#include "stm32f4xx_hal.h"


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

class uart_comm {
public:
    uart_comm(UART_HandleTypeDef *handler);
    void begin_uart();
    void handle_rx();

    // getter/setter
    float get_cx() const;
    float get_cy() const;
    bool get_isready() const;
    void reset_isready() ;


    friend void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

private:
    UART_HandleTypeDef *huart;
    float cx, cy;
    uint8_t rx_byte;
    bool is_ready;

    uint8_t idx;


    uint8_t rx_buffer[64];



    static uart_comm *instance; // Singleton 패턴용 인스턴스 포인터
};

#endif /* INC_UART_COMMUNICATION_HPP_ */
