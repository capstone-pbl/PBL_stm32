/*
 * actuator.hpp
 *
 *  Created on: Mar 22, 2026
 *      Author: zoon0
 */

#ifndef INC_ACTUATOR_HPP_
#define INC_ACTUATOR_HPP_
#include "main.h"
#include <string.h>
#include <stdio.h>


class ACT
{

public:
	ACT(TIM_HandleTypeDef* htim,
	UART_HandleTypeDef *huart,float ki);
	void Lifter_Up(void);
	void Lifter_Down(void);
	void Lifter_Stop(void);
	void Lifter_MoveTo(float target_mm);
private:
float COUNTS_PER_MM=411.4f;


int DEAD_ZONE = 2;
int LOOP_MS = 10;

int32_t prev_error = 0;

TIM_HandleTypeDef* htim;
UART_HandleTypeDef* huart;
float ki=0.0;
};

#endif /* INC_ACTUATOR_HPP_ */
