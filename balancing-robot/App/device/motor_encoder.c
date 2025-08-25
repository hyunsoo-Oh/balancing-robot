/*
 * motor_encoder.c
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#include "motor_encoder.h"
#include "pid.h"

#include "tim.h"
#include <math.h>

// ===== 하드웨어 핀/타이머 =====
#define MOTOR_R1_PORT		GPIOC
#define MOTOR_R2_PORT		GPIOA
#define MOTOR_L1_PORT		GPIOB
#define MOTOR_L2_PORT		GPIOB

#define MOTOR_R1_PIN		GPIO_PIN_3
#define MOTOR_R2_PIN		GPIO_PIN_2
#define MOTOR_L1_PIN		GPIO_PIN_5
#define MOTOR_L2_PIN		GPIO_PIN_4

#define MOTOR_TIM			&htim8
#define ENCODER_TIM_LEFT	&htim2
#define ENCODER_TIM_RIGHT	&htim5

#define PWM_CH_RIGHT   	TIM_CHANNEL_1
#define PWM_CH_LEFT  	TIM_CHANNEL_2

// 하드웨어 매핑 테이블
static GPIO_TypeDef* s_in1_port[MOTOR_COUNT] = { MOTOR_L1_PORT, MOTOR_R1_PORT };
static uint16_t      s_in1_pin [MOTOR_COUNT] = { MOTOR_L1_PIN , MOTOR_R1_PIN  };
static GPIO_TypeDef* s_in2_port[MOTOR_COUNT] = { MOTOR_L2_PORT, MOTOR_R2_PORT };
static uint16_t      s_in2_pin [MOTOR_COUNT] = { MOTOR_L2_PIN , MOTOR_R2_PIN  };

static TIM_HandleTypeDef* s_enc_tim[MOTOR_COUNT] = { ENCODER_TIM_LEFT, ENCODER_TIM_RIGHT };
static uint32_t           s_pwm_ch [MOTOR_COUNT] = { PWM_CH_LEFT     , PWM_CH_RIGHT      };

// ===== 엔코더/기구 스펙 =====
#define PPR					11		// 엔코더 PPR
#define GEAR_RATIO			30		// 모터:출력축 = 30:1
#define WHEEL_RADIUS_M    	0.034f  // 바퀴 반지름 [m]. 예: 지름 100mm -> 0.05

// ===== 내부 상태 =====
static volatile float s_rpm     [MOTOR_COUNT] = {0};
static volatile float s_thetaRad[MOTOR_COUNT] = {0}; // 출력축 누적 각도 [rad]
static volatile float s_distM   [MOTOR_COUNT] = {0}; // 누적 선형 거리 [m]
static int32_t        s_prevCnt [MOTOR_COUNT] = {0};

// ===== 파라미터 =====
static const float CPR = PPR * 4.0f * GEAR_RATIO;
static const float Ts  = 0.002f;  	// 3ms

// ===== 속도 PID 상태 =====
static pid_t pid_speed[MOTOR_COUNT];   // 모터별 속도 PID
static float target_rpm[MOTOR_COUNT] = {0.0f, 0.0f};  // 목표 RPM

static inline float clampf(float x, float lo, float hi){
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

void MOTOR_Init(void)
{
	HAL_TIM_PWM_Start(MOTOR_TIM, PWM_CH_LEFT );
	HAL_TIM_PWM_Start(MOTOR_TIM, PWM_CH_RIGHT);

	HAL_TIM_Encoder_Start(ENCODER_TIM_LEFT , TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(ENCODER_TIM_RIGHT, TIM_CHANNEL_ALL);

	__HAL_TIM_SET_COUNTER(ENCODER_TIM_LEFT , 0);
	__HAL_TIM_SET_COUNTER(ENCODER_TIM_RIGHT, 0);

	HAL_GPIO_WritePin(MOTOR_L1_PORT, MOTOR_L1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_L2_PORT, MOTOR_L2_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOTOR_R1_PORT, MOTOR_R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_R2_PORT, MOTOR_R2_PIN, GPIO_PIN_RESET);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		s_rpm[i] = 0.0f; s_thetaRad[i] = 0.0f; s_distM[i] = 0.0f; s_prevCnt[i] = 0;
	}
}

//   입력값 제한: u는 -1.0 ~ +1.0 범위만 허용
//   - 부호(sign) → 회전 방향
//   - 절댓값(magnitude) → 속도 비율(PWM 듀티)
void MOTOR_SetDuty_ID(motor_id_t id, float u)
{
    if (u > 1.0f)  u = 1.0f;
    if (u < -1.0f) u = -1.0f;

    GPIO_PinState in1, in2;
    float mag = (u >= 0) ? u : -u;

    if (u >= 0) { in1 = GPIO_PIN_SET;   in2 = GPIO_PIN_RESET; }
    else        { in1 = GPIO_PIN_RESET; in2 = GPIO_PIN_SET;   }

	// 방향핀 (모터별)
	HAL_GPIO_WritePin(s_in1_port[id], s_in1_pin[id], in1);
	HAL_GPIO_WritePin(s_in2_port[id], s_in2_pin[id], in2);

    // 한 주기 카운트 수 * 듀티 비율 + float → int 변환 시 반올림을 위한 상수
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(MOTOR_TIM);
    uint32_t ccr = (uint32_t)((arr + 1) * mag + 0.5f);
    __HAL_TIM_SET_COMPARE(MOTOR_TIM, s_pwm_ch[id], ccr);
}

int32_t ENCODER_GetCount_ID(motor_id_t id)
{
	return (int32_t)__HAL_TIM_GET_COUNTER(s_enc_tim[id]);
}

float ENCODER_GetRPM_ID     (motor_id_t id) { return s_rpm[id]; }
float ENCODER_GetAngleRad_ID(motor_id_t id) { return s_thetaRad[id]; }	// [rad]
float ENCODER_GetAngleDeg_ID(motor_id_t id) { return s_thetaRad[id] * 180.0f / (float)M_PI; }
float ENCODER_GetDistanceM_ID(motor_id_t id){ return s_distM[id]; }		// [m]

// 필요 시 원점복귀
void ENCODER_ResetPose_ID(motor_id_t id) { s_thetaRad[id] = 0.0f; s_distM[id] = 0.0f; }
void ENCODER_ResetPose_All(void)
{
	for (int i=0;i<MOTOR_COUNT;i++)
	{
		s_thetaRad[i]=0.0f;
		s_distM[i]=0.0f;
	}
}

// 10ms 주기에서 호출해 속도 계산
void ENCODER_Update(void)
{
	for (int id = 0; id < MOTOR_COUNT; id++)
	{
		static int32_t prev = 0;

		int32_t now  = ENCODER_GetCount_ID((motor_id_t)id);
		int32_t diff = now - s_prevCnt[id];
		s_prevCnt[id] = now;


		// 속도: 출력축 rps/rpm
		float rps = (float)diff / (CPR * Ts);
		s_rpm[id] = rps * 60.0f;

		// 각도: 출력축 누적 각도 [rad]
		float dTheta = 2.0f * (float)M_PI * ((float)diff / CPR);
		s_thetaRad[id] += dTheta;

		// 선형 거리: 바퀴 접지 기준 누적 거리 [m]
		s_distM[id]    += WHEEL_RADIUS_M * dTheta;  // s += R * dθ
	}
}

void MOTOR_SpeedPID_Init(float kp, float ki, float kd, float isum_limit, float output_limit)
{
    for (int id = 0; id < MOTOR_COUNT; id++)
    {
        PID_Init(&pid_speed[id], kp, ki, kd, isum_limit, output_limit); // out_limit은 듀티 상한(예: 1.0f)
        pid_speed[id].setpoint = 0.0f;
    }
}

void MOTOR_Speed_SetTargetRPM_ID(motor_id_t id, float rpm)
{
    if (id < 0 || id >= MOTOR_COUNT) return;
    target_rpm[id] = rpm;
    pid_speed[id].setpoint = rpm;   // pid.h의 setpoint 필드 사용
}

// 5ms마다 호출 (ENCODER_Update와 같은 주기)
void MOTOR_Speed_Update(void)
{
    for (int id = 0; id < MOTOR_COUNT; id++)
    {
        // 피드백: 현재 RPM
        float rpm_meas = ENCODER_GetRPM_ID(id);  // 이미 구현됨
        // PID 계산: 출력은 듀티 명령(-output_limit ~ +output_limit)
        float u = PID_Compute(&pid_speed[id], rpm_meas);
        u = -u;

//        extern UART_HandleTypeDef huart1;
//        char dbg[64];
//        int n = snprintf(dbg, sizeof(dbg), "u[%d]=%.2f, rpm=%.1f\r\n", id, u, rpm_meas);
//        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, n, 20);

        // 듀티 적용: MOTOR_SetDuty_ID는 -1.0~+1.0 입력 (부호=방향, 크기=듀티)
        // PID_Init의 output_limit을 1.0으로 주면 그대로 넣으면 됨.
        u = clampf(u, -1.0f, +1.0f);
        MOTOR_SetDuty_ID(id, u);
    }
}
