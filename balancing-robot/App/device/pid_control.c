/*
 * pid_control.c
 *
 *  Created on: Aug 12, 2025
 *      Author: USER
 */

#include "pid_control.h"

// 아주 단순한 clamp(원하면 제거 가능)
static inline float clampf(float x, float lo, float hi){
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

void pid_init(pid_t* p, float Kp, float Ki, float Kd, float dt, pid_dir_t dir)
{
    p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
    p->dt = (dt > 0.0f) ? dt : 1e-3f;
    p->setpoint = 0.0f;

    p->integ = 0.0f;
    p->prev_meas = 0.0f;
    p->direction = dir;
}

void pid_reset(pid_t* p, float meas_init)
{
    p->integ = 0.0f;
    p->prev_meas = meas_init;
}

float pid_update(pid_t* p, float meas)
{
    // 1) 오차
    float err = (float)p->direction * (p->setpoint - meas);

    // 2) D(meas) = -d(meas)/dt
    float d_meas = (meas - p->prev_meas) / p->dt;
    float D = -p->Kd * d_meas;

    // 3) I 적분
    p->integ += p->Ki * err * p->dt;

    // 4) 최종 출력
    float u = p->Kp * err + p->integ + D;

    // 5) 상태 갱신
    p->prev_meas = meas;
    return u;
}
