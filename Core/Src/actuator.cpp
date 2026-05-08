/*
 * actuator.cpp
 *
 *  Created on: Mar 22, 2026
 *      Author: zoon0
 */
#include "actuator.hpp"

ACT::ACT(TIM_HandleTypeDef* htim,
UART_HandleTypeDef* huart,float ki)
{
this->htim=htim;
this->huart=huart;
this->prev_error=0;
this->ki=ki;
}
void ACT::Lifter_Up(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

void ACT::Lifter_Down(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void ACT::Lifter_Stop(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

void ACT::Lifter_MoveTo(float target_mm)
{

        
        int32_t pos   = (int32_t)__HAL_TIM_GET_COUNTER(htim);

        
        float cur_error = target_mm - (float)pos/411.4;


        if(cur_error>DEAD_ZONE)
        {
            Lifter_Down();
        }
        else if(cur_error <-DEAD_ZONE)
        {
            Lifter_Up();
        }
        else
        {
            Lifter_Stop();
        }

        this->prev_error = cur_error;
        HAL_Delay(LOOP_MS);

}

