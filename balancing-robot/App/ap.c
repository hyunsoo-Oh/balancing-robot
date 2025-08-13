/*
 * ap.c
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#include "ap.h"
#include "usart.h"

#include "mpu6050.h"
#include "motor_encoder.h"

float accel[3], gyro[3];
float temp;

char msg[128];

void apInit(void)
{
	MPU6050_Init();

	MOTOR_Init();
	MOTOR_CTRL_Init();
	ENCODER_Update();

	HAL_Delay(1000);
	MOTOR_Angle_AbsDeg(-90.0f);
}

void apMain(void)
{
	uint32_t t10 	= HAL_GetTick();  // 10ms 제어 루프
	uint32_t t100 	= HAL_GetTick();  // 100ms 로깅

	while (1)
	{
		if ((int32_t)(HAL_GetTick() - t10) >= 10)
		{
			t10 += 10;

			// IMU 데이터 읽기
			MPU6050_GetAccelG(accel);
			MPU6050_GetGyroDPS(gyro);
			temp = MPU6050_GetTempCelsius();

			// 모터 제어
			ENCODER_Update();       // 센서 갱신
			MOTOR_Angle_Step();     // 속도 제어
		}
		if ((int32_t)(HAL_GetTick() - t100) >= 100)
		{
			t100 += 100;

			int32_t count = ENCODER_GetCount();
			float angle = ENCODER_GetAngleDeg();

			snprintf(msg, sizeof(msg), "Count: %ld, Angle: %.2f\r\n", (long)count, angle);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
//			snprintf(msg, sizeof(msg), "Roll=%f Pitch=%f Yaw=%f\r\n", gyro[0], gyro[1], gyro[2]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
		}
	}
}
