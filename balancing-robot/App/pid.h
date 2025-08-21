/*
 * pid.h
 *
 *  Created on: Aug 21, 2025
 *      Author: USER
 */

#ifndef PID_H_
#define PID_H_

#include "def.h"

// PID 구조체 (간단한 PID 구현)
typedef struct {
    float kp, ki, kd;           // PID 게인값
    float setpoint;             // 목표 RPM
    float integral;             // 적분 누적값
    float prev_error;           // 이전 오차 (미분용)
    float out_min, out_max;     // 출력 제한값
    float isum_min, isum_max;   // 적분 누적 제한값
} pid_t;

void PID_Init(pid_t *pid, float kp, float ki, float kd, float isum_limit, float output_limit);
float PID_Compute(pid_t *pid, float actual);

#endif /* PID_H_ */
