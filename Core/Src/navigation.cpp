/*
 * navigation.cpp
 *
 *  Created on: Mar 22, 2026
 *      Author: zoon0
 */
#include "navigation.hpp"
#include <algorithm>
//속도 계산->주행 알고리즘



Pure_pursuit::Pure_pursuit(TIM_HandleTypeDef *htim_encL,TIM_HandleTypeDef *htim_encR,delta_value &dv)
:L_PID({0,0,0}),R_PID({0,0,0}),L(0.39f),ld(0.3f)
//L:좌우 바퀴 간격  v:로봇 기본속도(m/s)  ld:lookahead distance
{
 this->htim_encL=htim_encL;
 this->htim_encR=htim_encR;
 this->dv=dv;

 this->prevL = __HAL_TIM_GET_COUNTER(htim_encL);
 this->prevR = __HAL_TIM_GET_COUNTER(htim_encR);

}

void Pure_pursuit::set_delta(delta_value&dv)
{
	dv.delta_encL = get_dist_L(); //좌 이동거리
	dv.delta_encR = get_dist_R(); //우 이동거리
	dv.delta_s     = (dv.delta_encR + dv.delta_encL) / 2.0f; //차동구동 선속도
	dv.delta_theta = (dv.delta_encR - dv.delta_encL) / (float)L;//차동구동 회전속도
	this->dv = dv;
	this->theta += dv.delta_theta;
} // 엔코더 읽어서 이미 일어난 움직임을 계산

void Pure_pursuit::update_theta(delta_value&dv, float cx, float cy)
{
        theta = atan2f(prev_cy-cy, cx-prev_cx);
        prev_cx = cx;
        prev_cy = cy;
}

WheelSpeed Pure_pursuit::calculate(float dist, float alpha)
{
    float v     = pos_con.kp_pos * dist;
    float omega = pos_con.kp_ang * alpha;
    float left_v  = v - omega * (float)L / 2.0f;
    float right_v = v + omega * (float)L / 2.0f;
    return WheelSpeed{left_v, right_v};
} //목표거리와 알파받아서 목표에서 멀면 속도 많이주고 가까우면 속도 느리게 조절해주는 함수


float Pure_pursuit::get_alpha(float cx,float cy,float tx,float ty)
{
   float alpha = atan2f(cy-ty,tx-cx)-theta;//alpha = atan2f(ty-cy,tx-cx) - theta

   while(alpha>M_PI)alpha -=2.0f*M_PI;
   while(alpha< -M_PI)alpha += 2.0f*M_PI;
   return alpha;
} //오차값을 리턴하는 함수. calculate메서드에서 오차값을 이용한 곱셈/나눗셈을 수행하여 p제어처럼 쓰게됨.




//이번 주기 펄스 수 = 현재 카운터 - 이전 카운터
//RPM = (펄스 수 / PPR) * (60 / dt)

float Pure_pursuit::get_dist_L(void)
{
 uint16_t curr = __HAL_TIM_GET_COUNTER(htim_encL);
 // 엔코더 현재 카운터 값 읽기
 int16_t delta = (int16_t)(curr - prevL);
 prevL=curr;
 //이번 주기 동안 바퀴가 돈 펄스 수= 현재 엔코더 카운터- 이전 엔코더 카운터
 return ((float)delta/PPR)*(2.0f*M_PI*0.05f);
 // delta/PPR = 몇바퀴 돌았는지, 반지름이 0.05f이므로 2pi*delta/PPR = 이번 주기 이동거리
}


float Pure_pursuit::get_dist_R(void)
{
	 uint16_t curr = __HAL_TIM_GET_COUNTER(htim_encR);
	 int16_t delta = (int16_t)(curr - prevR);
	 prevR=curr;
	 return ((float)delta/PPR)*(2.0f*M_PI*0.05f);
}



float Pure_pursuit::update_pid(PID_error & state, const vel_PID_Const &pid,float target,float current) //state: prev_error, error_sum 구조체, 하위제어기.
{

   float dt=0.02f;
   float error=target-current;
   state.error_sum+=error*dt;

   if(state.error_sum>0.135f)state.error_sum=0.135f;
   else if(state.error_sum<-0.135f)state.error_sum=-0.135f;

   float d_error=(error-state.prev_error)/dt;

   float output= pid.kp*error*200.0f + (pid.ki)*(state.error_sum)*200.0f + pid.kd*d_error;
   state.prev_error=error;


   if (output > 999.0f) output = 999.0f;
   else if (output < -999.0f) output = -999.0f;

   return output;
}



PID_error& Pure_pursuit::get_L_error()
{

return L_error;

}



PID_error& Pure_pursuit::get_R_error()
{

 return R_error;

}

void Pure_pursuit::set_pid_gain_L(const vel_PID_Const &pid)
{
 this->L_PID.kp=pid.kp;
 this->L_PID.ki=pid.ki;
 this->L_PID.kd=pid.kd;

}

void Pure_pursuit::set_pid_gain_R(const vel_PID_Const &pid)
{
 this->R_PID.kp=pid.kp;
 this->R_PID.ki=pid.ki;
 this->R_PID.kd=pid.kd;
}


float Pure_pursuit::get_theta()
{
	return theta;
}

void Pure_pursuit::set_pos_const(float kp_pos,float kp_ang)
{
   this->pos_con.kp_pos=kp_pos;
   this->pos_con.kp_ang=kp_ang;
}

void Pure_pursuit::set_prev_pos(float cx, float cy)
{
    prev_cx = cx;
    prev_cy = cy;
}
