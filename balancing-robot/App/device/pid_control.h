/*
 * pid_control.h
 *
 *  Created on: Aug 12, 2025
 *      Author: USER
 */

#ifndef DEVICE_PID_CONTROL_H_
#define DEVICE_PID_CONTROL_H_

#include "def.h"

typedef enum {
	PID_DIR_DIRECT 	= 1,
	PID_DIR_REVERSE = -1
} pid_dir_t;

typedef struct {
	float Kp, Ki, Kd;	// PID Gain
	float dt;			// 샘플링 주기 [s]

	float setpoint; 	// 목표 값

    float integ;        // 적분 상태
    float prev_meas;    // 이전 측정값

	pid_dir_t direction;	// DIRECT / REVERSE
} pid_t;

// 게인/샘플링/방향을 설정하고 내부 상태를 0으로 초기화
void  pid_init(pid_t* p, float Kp, float Ki, float Kd, float dt, pid_dir_t dir);

// 적분값을 0으로, 미분 기준점(prev_meas)을 현재 측정으로 동기화.
void  pid_reset(pid_t* p, float meas_init);

// 목표값 설정(인라인) - 외부에서 주기적으로 setpoint만 바꿔도 됨
static inline void pid_set_setpoint(pid_t* p, float sp) { p->setpoint = sp; }
float pid_update(pid_t* p, float meas);   // 반환값: 제어입력 u

#endif /* DEVICE_PID_CONTROL_H_ */
