/*
 * motor.hpp
 *
 *  Created on: Mar 22, 2026
 *      Author: zoon0
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_
#include "stm32f4xx_hal.h"

class run_motor
{

public:
   run_motor(TIM_HandleTypeDef *htim,uint32_t duty=100);

   uint32_t get_duty(uint32_t ch);
   void set_duty(uint32_t ch, uint32_t duty);

   void emergency_stop();

   void test_motor_L();
   void test_motor_R();

private:

uint32_t duty;
TIM_HandleTypeDef *htim;
};








#endif /* INC_MOTOR_HPP_ */
