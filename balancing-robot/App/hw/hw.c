/*
 * hw.c
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#include "hw.h"
#include "tim.h"
#include "usart.h"

#include "motor_encoder.h"
#include "balancer.h"

volatile uint32_t isr_tick = 0;

float roll_deg, pitch_deg;
float pitch_acc, pitch_gyr, pitch_cf;
float pitch;

void hwInit(void)
{
	HAL_TIM_Base_Start_IT(&htim10);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
//    	float accel[3], gyro[3];

//		MPU6050_GetAccelG(accel);
//		MPU6050_GetGyroDPS(gyro);
//
//        roll_deg  = BALANCE_Angle_GetRollAcc(accel[1], accel[2]);

//        pitch_acc = BALANCE_Angle_GetPitchAcc(accel[0], accel[1], accel[2]);
//        pitch_gyr = BALANCE_Angle_GetPitchGyr(gyro[1]);
//        pitch_cf  = BALANCE_Angle_GetPitchCF(accel[0], accel[1], accel[2], gyro[1]);
//        pitch = BALANCE_UpdatePitch(accel[0], accel[0], accel[0], gyro[0]);

        ENCODER_Update();
		Balance_Update();

    }
}
