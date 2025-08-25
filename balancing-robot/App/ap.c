/*
 * ap.c
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#include "ap.h"
#include "usart.h"
#include "tim.h"

#include "mpu6050.h"
#include "motor_encoder.h"
#include "balancer.h"

extern float acc_g[3], gy_dps[3];

extern float roll_deg, pitch_deg;
extern float pitch_acc, pitch_gyr, pitch_cf;
float temp = 0.0f;

char msg[128];

//static float g_target_rpm = -220.0f;

// UART로 받은 문자열을 저장할 버퍼
static char uart_rx_buffer[128];
static volatile bool uart_cmd_ready = false;

void apInit(void)
{
	MPU6050_Init();

	MOTOR_Init();
	ENCODER_Update();

//	MOTOR_SpeedPID_Init(0.05f,    // kp: 비례 게인:
//	                    0.20f,    // ki: 적분 게인
//	                    0.00f,    // kd: 미분 게인
//	                    1.0f,   // isum_limit: 적분 제한
//	                    1.0f);   // output_limit: 출력 제한

	BALANCE_Init();
	BALANCE_Enable(true);             // 시작 On
	BALANCE_SetTargetSpeed(0.0f);     // 정지부터

	HAL_Delay(1000);

	HAL_TIM_Base_Start_IT(&htim10);
}

void apMain(void)
{
    uint32_t t_last = HAL_GetTick();

//    // UART 인터럽트 수신 시작
//    uint8_t rx_char;
//    HAL_UART_Receive_IT(&huart1, &rx_char, 1);

	while (1)
	{
//		snprintf(msg, sizeof(msg),
//			 "ACC: X=%.2f Y=%.2f Z=%.2f | GYRO: X=%.2f Y=%.2f Z=%.2f\r\n",
//			 acc_g[0], acc_g[1], acc_g[2],
//			 gy_dps[0], gy_dps[1], gy_dps[2]
//		);
//		HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);

//		// 100ms마다 상태 출력
//		if (HAL_GetTick() - t_last >= 100)
//		{
//			t_last = HAL_GetTick();
//
//			float pitch_angle = BALANCE_GetCurrentAngle();
//			bool is_enabled = BALANCE_IsEnabled();
//
//			float rpmL = ENCODER_GetRPM_ID(MOTOR_LEFT);
//			float rpmR = ENCODER_GetRPM_ID(MOTOR_RIGHT);
//
//			int n = snprintf(msg, sizeof(msg),
//						   "Pitch: %6.2f°, RPM L:%6.1f R:%6.1f [%s]\r\n",
//						   pitch_angle, rpmL, rpmR,
//						   is_enabled ? "ON" : "OFF");
//			HAL_UART_Transmit(&huart1, (uint8_t*)msg, n, 50);
//		}
//		float acc_g[3], gy_dps[3];
//		MPU6050_GetAccelG(acc_g);
//		MPU6050_GetGyroDPS(gy_dps);
//
//		snprintf(msg, sizeof(msg),
//			 "ACC: X=%.2f Y=%.2f Z=%.2f | GYRO: X=%.2f Y=%.2f Z=%.2f | TEMP=%.2f\r\n",
//			 acc_g[0], acc_g[1], acc_g[2],
//			 gy_dps[0], gy_dps[1], gy_dps[2],
//			 temp
//		);
//
//		HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
	}
}

//t_last = HAL_GetTick();
//
//float rpmL = ENCODER_GetRPM_ID(MOTOR_LEFT);
//float rpmR = ENCODER_GetRPM_ID(MOTOR_RIGHT);
//float rpmAvg = 0.5f * (rpmL + rpmR);
//
//MOTOR_Speed_SetTargetRPM_ID(MOTOR_LEFT,  g_target_rpm);
//MOTOR_Speed_SetTargetRPM_ID(MOTOR_RIGHT, g_target_rpm);
//
//// 예: "Target: 200 RPM, Real: 203.1 RPM"
//int n = snprintf(msg, sizeof(msg),
//                 "Target: %.0f RPM, Real: %.1f RPM (L:%.1f, R:%.1f)\r\n",
//                 g_target_rpm, rpmAvg, rpmL, rpmR);
//HAL_UART_Transmit(&huart1, (uint8_t*)msg, n, 50);

//		snprintf(msg, sizeof(msg), "pitch_acc:%.2f, pitch_gyr:%.2f, pitch_cf:%.2f\r\n", pitch_acc, pitch_gyr, pitch_cf);
//		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

//		snprintf(msg, sizeof(msg),
//			 "ACC: X=%.2f Y=%.2f Z=%.2f | GYRO: X=%.2f Y=%.2f Z=%.2f | TEMP=%.2f\r\n",
//			 accel[0], accel[1], accel[2],
//			 gyro[0], gyro[1], gyro[2],
//			 temp
//		);
//
//		HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
