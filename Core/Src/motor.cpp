/*
 * motor.cpp
 *
 *  Created on: Mar 22, 2026
 *      Author: zoon0
 */
#include "stm32f4xx_hal.h"
#include "motor.hpp"

  run_motor::run_motor(TIM_HandleTypeDef *htim,uint32_t duty)
   {
      this->htim = htim;
	  this->duty = (duty <=100) ? duty : 50;
   }



   uint32_t run_motor::get_duty(uint32_t ch)
   {
     return __HAL_TIM_GET_COMPARE(htim,ch);
   }

   void run_motor::emergency_stop()
   {
	   __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
	   __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
	   __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
	   __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0);
   }

   void run_motor::test_motor_L()
   {
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1,duty);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2,0);
      HAL_Delay(3000);
   HAL_TIM_PWM_Stop(htim,TIM_CHANNEL_1);
   HAL_TIM_PWM_Stop(htim,TIM_CHANNEL_2);

   }

  void run_motor::test_motor_R()
   {
	 HAL_TIM_PWM_Start(htim,TIM_CHANNEL_3);
     HAL_TIM_PWM_Start(htim,TIM_CHANNEL_4);


     __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,duty);
     __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4,0);
          HAL_Delay(3000);
       HAL_TIM_PWM_Stop(htim,TIM_CHANNEL_3);
       HAL_TIM_PWM_Stop(htim,TIM_CHANNEL_4);

   }

  void run_motor::set_duty(uint32_t ch, uint32_t duty) {
      if(duty > 100)duty=100;
	  this->duty = duty;

      __HAL_TIM_SET_COMPARE(htim, ch, duty);
  }

