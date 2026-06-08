/*
 * navigation.hpp
 *
 *  Created on: Mar 22, 2026
 *      Author: zoon0
 */

#ifndef INC_NAVIGATION_HPP_
#define INC_NAVIGATION_HPP_
#include "stm32f4xx_hal.h"
#include "motor.hpp"
#include <cmath>
#include "actuator.hpp"
#include "uart_communication.hpp"

//속도 계산->주행 알고리즘

struct delta_value {
	float delta_encL, delta_encR, delta_s, delta_theta;
};
struct WheelSpeed {
	float left, right;
};

struct PID_error{

  float error_sum=0.0f,prev_error=0.0f;

};
struct vel_PID_Const
{
 float kp,ki,kd;
};

struct pos_const
{
  float kp_pos;
  float kp_ang;
};
class Pure_pursuit {
public:
	Pure_pursuit(TIM_HandleTypeDef *htim_encL, TIM_HandleTypeDef *htim_encR,
			delta_value &dv);

	void set_delta(delta_value &dv);
	void update_theta(delta_value &dv,float cx,float cy);
	WheelSpeed calculate(float dist, float alpha);
    float get_theta();

//cx,cy:현재 로봇위치 (UWB가 실시간으로 줌)
//tx,ty: 목표위치, 앱에서 가라고 명령받는 좌표(미리 정의)
//alpha: heading 기준 목표까지 상대각(로봇이 바라보는 방향 기준으로 목표가 몇 도에 있는지)
//alpha = atan2(ty-cy,tx-cx) - theta

	float get_alpha(float cx, float cy, float tx, float ty);
	float get_dist_L(void);
	float get_dist_R(void);
    float update_pid(PID_error & state, const vel_PID_Const &pid,float target,float current); //state: prev_error, error_sum 구조체, 하위제어기.


    PID_error& get_L_error();
    PID_error& get_R_error();
    void set_pid_gain_L(const vel_PID_Const& pid);
    void set_pid_gain_R(const vel_PID_Const&pid);
	vel_PID_Const L_PID,R_PID;
	void set_pos_const(float kp_pos,float kp_ang);
	void set_prev_pos(float cx, float cy);
	float theta = 0.0f;
	float prev_cx=0.0f,prev_cy=0.0f;


private:
	TIM_HandleTypeDef *htim_encL;
	TIM_HandleTypeDef *htim_encR;

	uint16_t prevL = 0;
	uint16_t prevR = 0;
	const double L; //L:바퀴 간격
	double v, ld; //v:로봇 중심속도, w:회전 각속도, ld: look ahead distance
	/* ld 작으면  가까운 목표 봄
	 조향이 민감하게 반응
	 곡선 구간에서 잘 따라가지만 진동/떨림 발생 가능

	 ld 크면     멀리 있는 목표 봄
	 조향이 부드러움
	 직선 구간에서 안정적이지만
	 곡선 구간에서 늦게 반응

	 ld 작음   ->발 앞만 보고 걷는 사람 (민감)
	 ld 큼    ->멀리 보고 걷는 사람 (안정적)
	 */

	delta_value dv{};

	PID_error L_error,R_error;
    pos_const pos_con{0.1f,1.5f};
	const float PPR = 26 * 4* 180;
};

#endif /* INC_NAVIGATION_HPP_ */
