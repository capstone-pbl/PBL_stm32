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
:L(0.15f),v(0.5f),ld(0.3f),kp(19.0),ki(37),kd(0.18)
//L:좌우 바퀴 간격  v:로봇 기본속도(m/s)  ld:lookahead distance
{
 this->htim_encL=htim_encL;
 this->htim_encR=htim_encR;
 this->dv=dv;

 this->prevL = __HAL_TIM_GET_COUNTER(htim_encL);
 this->prevR = __HAL_TIM_GET_COUNTER(htim_encR);

}

//좌우 이동거리 차이로 로봇이 얼마나 회전했는지 계산
void Pure_pursuit::set_delta(delta_value&dv)
{
	dv.delta_encL = get_dist_L(); //좌 이동거리
	dv.delta_encR = get_dist_R(); //우 이동거리
	dv.delta_s     = (dv.delta_encR + dv.delta_encL) / 2.0f; //중심 이동거리
	dv.delta_theta = (dv.delta_encR - dv.delta_encL) / (float)L;//회전거리
	this->dv = dv;
}
void Pure_pursuit::Odometry(delta_value&dv)
{
 this->theta += dv.delta_theta; //heading 누적
 this-> x += dv.delta_s*cosf(this->theta);// x위치 누적
 this-> y += dv.delta_s*sinf(this->theta);// y위치 누적
}//

WheelSpeed Pure_pursuit::calculate(float cx,float cy,float tx, float ty,float alpha)
{
	float actual_ld = std::max(ld, 0.1);
	// ld가 너무 작으면 0으로 나뉘는 에러 방지
	float kappa = (2.0f * sinf(alpha)) / (float)actual_ld; //곡률 kappa = 2sina/ ld : 얼마나 휘어야 하는지
	//alpha: 로봇 heading 기준 목표까지 상대각
	//alpha가 작으면 목표가 정면에 있다는 뜻이므로 조금만 꺾음
	//alpha가 크면 목표와 각도 차이가 커서 많이 꺾음
	float omega  = (float)v * kappa;//각속도 w= 선속도*곡률 = v*k;
	float left_v  = (float)v - omega * ((float)L / 2.0f);
	float right_v = (float)v + omega * ((float)L / 2.0f);

	return WheelSpeed{left_v, right_v};

}


float Pure_pursuit::get_alpha(float cx,float cy,float tx,float ty)
{
   float alpha = atan2f(ty-cy,tx-cx)-theta;//alpha = atan2f(ty-cy,tx-cx) - theta

   while(alpha>M_PI)alpha -=2.0f*M_PI;
   while(alpha< -M_PI)alpha += 2.0f*M_PI;
   return alpha;
}




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



float Pure_pursuit::update_pid(PID_error & state,float target,float current)
{

   float dt=0.02f;
   float error=target-current;
   state.error_sum+=error*dt;

//   if(state.error_sum>10.0f)state.error_sum=10.0f;
//   else if(state.error_sum<-10.0f)state.error_sum=-10.0f;

   float d_error=(error-state.prev_error)/dt;

   float output= Pure_pursuit::kp*error*200.0f + (Pure_pursuit::ki)*(state.error_sum)*200.0f + kd*d_error;
   state.prev_error=error;


//   if (output > 100.0f) output = 100.0f;
//   else if (output < -100.0f) output = -100.0f;

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

void Pure_pursuit::set_pid_gain(float kp,float ki,float kd)
{

this->kp=kp;
this->ki=ki;
this->kd=kd;

}

