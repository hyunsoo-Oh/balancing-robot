/*
 * motor_encoder.c
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#include "motor_encoder.h"

#include "tim.h"
#include <math.h>
#include "pid_control.h"

// ===== 하드웨어 핀/타이머 =====
#define MOTOR_C1_PORT		GPIOA
#define MOTOR_C2_PORT		GPIOC
//#define MOTOR_C1_PORT		GPIOB
//#define MOTOR_C2_PORT		GPIOB

#define MOTOR_C1_PIN		GPIO_PIN_2
#define MOTOR_C2_PIN		GPIO_PIN_3
//#define MOTOR_C1_PIN		GPIO_PIN_4
//#define MOTOR_C2_PIN		GPIO_PIN_5


#define MOTOR_TIM			&htim8
#define ENCODER_TIM_LEFT	&htim2
#define ENCODER_TIM			&htim2


// ===== 엔코더/기구 스펙 =====
#define PPR					11		// 엔코더 PPR
#define GEAR_RATIO			30		// 모터:출력축 = 30:1
#define WHEEL_RADIUS_M    	0.034f  // 바퀴 반지름 [m]. 예: 지름 100mm -> 0.05

// ===== 내부 상태 =====
static volatile float s_rpm      = 0.0f;
static volatile float s_thetaRad = 0.0f;   // 출력축 누적 각도 [rad]
static volatile float s_distM    = 0.0f;   // 누적 선형 거리 [m]

// ===== 파라미터 =====
static const float CPR = PPR * 4.0f * GEAR_RATIO;
static const float Ts  = 0.01f;  	// 10ms

// ===== 제어 파라미터/상태 =====
#define DUTY_MAX_SPEED   	1.0f   // PWM 듀티 상한 (절대값)
#define DUTY_MAX_ANGLE  	0.8f   // PWM 듀티 상한 (절대값)
#define STOP_RPM        	5.0f    // 정지로 보는 속도 임계
#define ANGLE_TOL_DEG   	1.0f    // 각도 목표 허용오차(±3도)

static pid_t s_pid_speed;     // 속도 PI: rpm -> duty
static pid_t s_pid_angle;     // 각도 PD: deg -> duty
static bool  s_angle_active = false; // 각도 이동 활성화 여부

// 유틸리티
// 각도 제어 시 오버슈트 현상 발생 -> 90° 도달 후 튕겨서 70°
#define ANG_SOFT_DEG   10.0f   // 목표 ±10° 안에서는 듀티를 선형으로 줄이기
#define ANG_LOCK_DEG    2.0f   // 목표 교차 시 ±2° 안이면 즉시 정지

static inline float clampf(float x, float lo, float hi){
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

void MOTOR_Init(void)
{
	HAL_TIM_PWM_Start(MOTOR_TIM, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(ENCODER_TIM, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(ENCODER_TIM, 0);

	HAL_GPIO_WritePin(MOTOR_C1_PORT, MOTOR_C1_PIN, RESET);
	HAL_GPIO_WritePin(MOTOR_C2_PORT, MOTOR_C2_PIN, RESET);
}

//   입력값 제한: u는 -1.0 ~ +1.0 범위만 허용
//   - 부호(sign) → 회전 방향
//   - 절댓값(magnitude) → 속도 비율(PWM 듀티)
void MOTOR_SetDuty(float u)
{
    if (u > 1.0f)  u = 1.0f;
    if (u < -1.0f) u = -1.0f;

    GPIO_PinState in1, in2;
    float mag = (u >= 0) ? u : -u;

    if (u >= 0) { in1 = GPIO_PIN_SET;   in2 = GPIO_PIN_RESET; }
    else        { in1 = GPIO_PIN_RESET; in2 = GPIO_PIN_SET;   }

    HAL_GPIO_WritePin(MOTOR_C1_PORT, MOTOR_C1_PIN, in1);
    HAL_GPIO_WritePin(MOTOR_C2_PORT, MOTOR_C2_PIN, in2);

    // 한 주기 카운트 수 * 듀티 비율 + float → int 변환 시 반올림을 위한 상수
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(MOTOR_TIM);
    uint32_t ccr = (uint32_t)((arr + 1) * mag + 0.5f);
    __HAL_TIM_SET_COMPARE(MOTOR_TIM, TIM_CHANNEL_1 | TIM_CHANNEL_2, ccr);
}

int32_t ENCODER_GetCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(ENCODER_TIM);
}

float ENCODER_GetRPM(void)      { return s_rpm; }
float ENCODER_GetAngleRad(void) { return s_thetaRad; }     // [rad]
float ENCODER_GetAngleDeg(void) { return s_thetaRad * 180.0f / (float)M_PI; }
float ENCODER_GetDistanceM(void){ return s_distM; }        // [m]

// 필요 시 원점복귀
void ENCODER_ResetPose(void) { s_thetaRad = 0.0f; s_distM = 0.0f; }

// 10ms 주기에서 호출해 속도 계산
void ENCODER_Update(void)
{
    static int32_t prev = 0;

    int32_t now  = ENCODER_GetCount();
    int32_t diff = now - prev;
    prev = now;

    // 속도: 출력축 rps/rpm
    float rps  = (float)diff / (CPR * Ts);
    s_rpm = rps * 60.0f;

    // 각도: 출력축 누적 각도 [rad]
    float dTheta = 2.0f * (float)M_PI * ((float)diff / CPR);
    s_thetaRad += dTheta;

    // 선형 거리: 바퀴 접지 기준 누적 거리 [m]
    s_distM += WHEEL_RADIUS_M * dTheta;  // s += R * dθ
}

// 제어(원하는 속도 / 원하는 각도)
//  - MOTOR_CTRL_Init(): 앱 시작 시 1회. PID 게인/주기 설정 + 상태 동기화
//  - SPEED_*: 목표 rpm 지정 및 매주기 계산 → duty
//  - ANGLE_*: 목표 각도(절대/상대) 지정 및 매주기 계산 → duty, 완료시 true
void MOTOR_CTRL_Init(void)
{
    // 주의: dt=0.01s (10ms) — ENCODER_Update 주기와 반드시 동일!
    pid_init(&s_pid_speed, /*Kp*/0.10f, /*Ki*/1.20f, /*Kd*/0.00f, /*dt*/Ts, PID_DIR_DIRECT); // 속도 PI
    pid_init(&s_pid_angle, /*Kp*/0.40f, /*Ki*/0.00f, /*Kd*/0.02f, /*dt*/Ts, PID_DIR_DIRECT); // 각도 PD

    // 초기 목표를 현재 상태에 맞춰 세팅(부드러운 시작)
    pid_set_setpoint(&s_pid_speed, ENCODER_GetRPM());
    pid_set_setpoint(&s_pid_angle, ENCODER_GetAngleDeg());
    pid_reset(&s_pid_speed, 0.0f);
    pid_reset(&s_pid_angle, ENCODER_GetAngleDeg());
}

// ===== 원하는 속도 제어 (회전 속도 유지) =====
// 목표 속도로 회전
void MOTOR_SetRPM(float rpm_target)
{
    pid_set_setpoint(&s_pid_speed, rpm_target);
}

// 10ms마다 호출: 속도 PI → duty
void MOTOR_Speed_Step(void)
{
    float rpm_meas = ENCODER_GetRPM();
    float duty     = pid_update(&s_pid_speed, rpm_meas);
    duty = clampf(duty, -DUTY_MAX_SPEED, DUTY_MAX_SPEED);
    MOTOR_SetDuty(duty);
}

// ===== 원하는 각도 회전 =====
// 절대 각도 목표 (예: 90도 위치로 회전)
void MOTOR_Angle_AbsDeg(float target_deg)
{
    pid_set_setpoint(&s_pid_angle, target_deg);
    pid_reset(&s_pid_angle, ENCODER_GetAngleDeg()); // 미분 기준 동기화
    s_angle_active = true;
}

// 현재 각도 기준 상대 회전 (예: +45도, -30도)
void MOTOR_Angle_GoRelDeg(float delta_deg)
{
    float target = ENCODER_GetAngleDeg() + delta_deg;
    MOTOR_Angle_AbsDeg(target);
}

// 10ms마다 호출: 각도 PD → duty, 완료시 true 반환
bool MOTOR_Angle_Step(void)
{
    if (!s_angle_active) return true;

    static float prev_err = 0.0f;      // 직전 각도 오차(교차 감지용)

    float ang_meas = ENCODER_GetAngleDeg();
    float err      = s_pid_angle.setpoint - ang_meas;    // [deg]
    float aerr     = fabsf(err);

    // PD 계산: D(measurement) 사용 → 자연스러운 감쇠
    float duty = pid_update(&s_pid_angle, ang_meas);

    // 소프트 랜딩: 목표 근처(±ANG_SOFT_DEG)에서 듀티 선형 축소
    if (aerr < ANG_SOFT_DEG) {
        float scale = aerr / ANG_SOFT_DEG;   // 0..1
        duty *= scale;
    }

    duty = clampf(duty, -DUTY_MAX_ANGLE, DUTY_MAX_ANGLE);
    MOTOR_SetDuty(duty);

    // 완료 판정: 오차 작고, 속도도 거의 0
    float rpm = fabsf(ENCODER_GetRPM());
    if (aerr <= ANGLE_TOL_DEG && rpm < STOP_RPM) {
        MOTOR_SetDuty(0.0f);
        s_angle_active = false;
        prev_err = err;
        return true;
    }

    // 교차 락: 목표를 한 번 넘었고 ±ANG_LOCK_DEG 안이면 바로 멈춤
    if ((prev_err * err) < 0.0f && aerr <= ANG_LOCK_DEG) {
        MOTOR_SetDuty(0.0f);
        s_angle_active = false;
        prev_err = err;
        return true;
    }

    prev_err = err;
    return false;
}
