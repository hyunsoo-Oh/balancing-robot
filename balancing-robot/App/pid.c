/*
 * pid.c
 *
 *  Created on: Aug 21, 2025
 *      Author: USER
 */

#include "pid.h"

#include <math.h>

// 샘플링 시간 (ENCODER_Update 주기와 동일)
#define PID_TS          0.002f   // 2ms

// PID 초기화 함수
void PID_Init(pid_t *pid, float kp, float ki, float kd, float isum_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral 	= 0.0f;	// 적분 누적값
    pid->prev_error = 0.0f; // 이전 오차 (미분용)

    pid->out_min = -output_limit;
    pid->out_max = output_limit;

    pid->isum_min = -isum_limit;
    pid->isum_max = isum_limit;
}

// PID 계산 함수
float PID_Compute(pid_t *pid, float actual)
{
    float error = pid->setpoint - actual;

    // 비례(P) 계산
    float p_out = pid->kp * error;

    // 적분(I) 계산 (와인드업 방지)
    pid->integral += pid->ki * error * PID_TS;
    if (pid->integral > pid->isum_max) pid->integral = pid->isum_max;
    if (pid->integral < pid->isum_min) pid->integral = pid->isum_min;
    float i_out = pid->integral;

    // 미분(D) 계산 (오차 기반)
    float d_out = pid->kd * (error - pid->prev_error) / PID_TS;
    pid->prev_error = error;

    // 총 출력 계산
    float output = p_out + i_out + d_out;

    // 출력 제한 적용
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}
