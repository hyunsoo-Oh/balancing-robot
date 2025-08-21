/*
 * motor_encoder.h
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#ifndef DEVICE_MOTOR_ENCODER_H_
#define DEVICE_MOTOR_ENCODER_H_

#include "def.h"

typedef enum {
	MOTOR_LEFT = 0,
	MOTOR_RIGHT = 1,
	MOTOR_COUNT = 2
} motor_id_t;

void MOTOR_Init(void);

// -1.0 ~ +1.0 입력 (부호=방향, 크기=듀티)
void  MOTOR_SetDuty_ID(motor_id_t id, float u);

// 엔코더 카운트
int32_t ENCODER_GetCount_ID(motor_id_t id);

// 엔코더로 속도/각도/거리 읽기
float ENCODER_GetRPM_ID(motor_id_t id);
float ENCODER_GetAngleRad_ID(motor_id_t id);
float ENCODER_GetAngleDeg_ID(motor_id_t id);
float ENCODER_GetDistanceM_ID(motor_id_t id);

void  ENCODER_ResetPose_ID(motor_id_t id);
void  ENCODER_ResetPose_All(void);

void ENCODER_Update(void);

void  MOTOR_SpeedPID_Init(float kp, float ki, float kd, float isum_limit, float output_limit);
void  MOTOR_Speed_SetTargetRPM_ID(motor_id_t id, float rpm);
void  MOTOR_Speed_Update(void);  // 5ms 주기에서 호출

#endif /* DEVICE_MOTOR_ENCODER_H_ */
