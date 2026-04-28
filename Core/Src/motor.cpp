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
	  this->duty = (duty <=500) ? duty : 500;
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
    HAL_TIM_PWM_Start(this->htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(this->htim,TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_1,this->duty);
    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_2,0); //정방향

    HAL_Delay(1000);

    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_2,0);//정지

    HAL_Delay(500);

    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_2,this->duty);//역방향

    HAL_Delay(1000);

    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_2,0);//정지

    HAL_Delay(500);

   HAL_TIM_PWM_Stop(this->htim,TIM_CHANNEL_1);
   HAL_TIM_PWM_Stop(this->htim,TIM_CHANNEL_2);


   }

  void run_motor::test_motor_R()
   {
	 HAL_TIM_PWM_Start(this->htim,TIM_CHANNEL_3);
     HAL_TIM_PWM_Start(this->htim,TIM_CHANNEL_4);


     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_3,this->duty);
     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_4,0); //정방향


     HAL_Delay(1000);

     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_3,0);
     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_4,0);//정지


     HAL_Delay(500);

     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_3,0);
     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_4,this->duty); //역방향

     HAL_Delay(1000);

     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_3,0);
     __HAL_TIM_SET_COMPARE(this->htim, TIM_CHANNEL_4,0);//정지

     HAL_Delay(500);

       HAL_TIM_PWM_Stop(this->htim,TIM_CHANNEL_3);
       HAL_TIM_PWM_Stop(this->htim,TIM_CHANNEL_4);

   }

  void run_motor::set_duty(uint32_t ch, uint32_t duty) {
      if(duty > 500)duty=500;
	  this->duty = duty;

      __HAL_TIM_SET_COMPARE(this->htim, ch, duty);
  }

