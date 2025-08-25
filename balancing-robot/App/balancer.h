/*
 * balancer.h
 *
 *  Created on: Aug 13, 2025
 *      Author: USER
 */

#ifndef BALANCER_H_
#define BALANCER_H_

#include "def.h"

float BALANCE_Angle_GetRollAcc(float ay, float az);
float BALANCE_Angle_GetPitchAcc(float ax, float ay, float az);
float BALANCE_Angle_GetPitchGyr(float gy);
float BALANCE_Angle_GetPitchCF(float ax, float ay, float az, float gy);

float BALANCE_UpdatePitch(float ax, float ay, float az, float gy);

void  BALANCE_Init(void);
void  BALANCE_Enable(bool enable);
bool  BALANCE_IsEnabled(void);

void  BALANCE_SetTargetSpeed(float speed_ms);     // m/s
float BALANCE_GetCurrentAngle(void);              // deg

// 5ms 주기(ENCODER_Update와 동일 ISR)에서 호출
void  BALANCE_Update(void);

// 런타임 튠
void  Balance_SetAnglePID(float kp, float ki, float kd);
void  Balance_SetSpeedPID(float kp, float ki, float kd);

#endif /* BALANCER_H_ */
