/*
 * motor_encoder.h
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#ifndef DEVICE_MOTOR_ENCODER_H_
#define DEVICE_MOTOR_ENCODER_H_

#include "def.h"

void MOTOR_Init(void);

// -1.0 ~ +1.0 입력 (부호=방향, 크기=듀티)
void MOTOR_SetDuty(float u);

// 엔코더 카운트
int32_t ENCODER_GetCount(void);

// 엔코더로 속도/각도/거리 읽기
float   ENCODER_GetRPM(void);
float ENCODER_GetAngleRad(void);
float ENCODER_GetAngleDeg(void);
float ENCODER_GetDistanceM(void);

void ENCODER_Update(void);

void MOTOR_CTRL_Init(void);

void MOTOR_SetRPM(float rpm_target);
void MOTOR_Speed_Step(void);

void MOTOR_Angle_AbsDeg(float target_deg);	// 절대 각도 목표 (예: 90도 위치로 회전)
void MOTOR_Angle_GoRelDeg(float delta_deg); // 현재 각도 기준 상대 회전 (예: +45도, -30도)
bool MOTOR_Angle_Step(void); // 10ms마다 호출: 각도 PD → duty, 완료시 true 반환
#endif /* DEVICE_MOTOR_ENCODER_H_ */
