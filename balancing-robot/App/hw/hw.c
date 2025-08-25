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

float acc_g[3], gy_dps[3];

float roll_deg, pitch_deg;
float pitch_acc, pitch_gyr, pitch_cf;
float pitch;

static char msg[128];

void hwInit(void)
{

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
        BALANCE_Update();
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1)	return;
}
