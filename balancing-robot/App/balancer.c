/*
 * balancer.c
 *
 *  Created on: Aug 13, 2025
 *      Author: USER
 */


#include "balancer.h"

#include <math.h>
#include "motor_encoder.h"
#include "mpu6050.h"
#include "pid.h"

// ===== 밸런스 제어 파라미터 =====
// 자이로 pitch 축 인덱스(보통 Y=1), 필요시 부호 뒤집기
#define GY_PITCH_IDX   1
#define GY_PITCH_SIGN  (+1.0f)

// ===== 제한/데드밴드 =====
#define MAX_TILT_ANGLE   45.0f   // 안전 각도 한계[deg]
#define DEADBAND_ANGLE    1.0f   // ±1deg 이하는 무시

// ===== PID 파라미터(시작점) =====
// 각도(inner): 출력=듀티
#define ANG_KP    	2.05f
#define ANG_KI    	0.0f
#define ANG_KD    	0.2f
#define ANG_ISUM  	0.0f
#define ANG_OUT   	0.83f   // 듀티 제한

// 속도(outer): 출력=목표 pitch(deg 등가)
#define SPD_KP   	0.05f
#define SPD_KI   	0.30f
#define SPD_KD   	0.00f
#define SPD_ISUM 	1.0f
#define SPD_OUT  	8.0f

// ===== 각도 오차 제거 =====
#define ROLL_ACC_OFFSET			-1.54
#define PITCH_ACC_OFFSET		3.92
#define YAW_ACC_OFFSET			3

// ===== 제어 상태 =====
static float pitch_deg = 0.0f;
static float pitch_rad = 0.0f;

// ===== 내부 상태 =====
static bool        s_enabled = false;
static float       s_target_speed = 0.0f;    // m/s
static float       s_pitch_deg = 0.0f;       // 상보필터 추정(deg)

// PID 인스턴스(네 PID 타입/API)
static pid_t s_pid_angle;   // 각도 루프: (meas=deg, set=0deg) → duty
static pid_t s_pid_speed;   // 속도 루프: (meas=m/s, set=target) → pitch_cmd(deg)

static const float Ts  = 0.002f;  	// 2ms

// 상보 필터 파라미터
static const float fc = 1.0f;          // 컷오프 주파수 [Hz] (0.5~2.0Hz에서 튜닝)
static const float tau   = 1.0f / (2.0f * (float)M_PI * fc);
static const float alpha = tau / (tau + Ts);

static float clamp_float(float value, float min_val, float max_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

static float deadband_filter(float input, float threshold)
{
    if (fabs(input) < threshold) return 0.0f;
    return input;
}

// Roll 계산 (deg 단위)
float BALANCE_Angle_GetRollAcc(float ay, float az)
{
    float theta_rad = atan2f(-ay, az);
    return theta_rad * (180.0f / (float)M_PI) + ROLL_ACC_OFFSET;
}

// Pitch 계산 (deg 단위)
float BALANCE_Angle_GetPitchAcc(float ax, float ay, float az)
{
    float denom     = sqrtf(ay*ay + az*az);    // Roll 영향 제거
    float theta_rad = atan2f(-ax, denom);      // 앞뒤 기울기
    return theta_rad * (180.0f / (float)M_PI) + PITCH_ACC_OFFSET;
}

// Pitch 계산 (deg 단위)
float BALANCE_Angle_GetPitchGyr(float gy)
{
	static float pitch_deg = 0.0f;
	pitch_deg += gy * Ts;
	return pitch_deg;
}

// Pitch 계산 (deg 단위)
float BALANCE_Angle_GetPitchCF(float ax, float ay, float az, float gy)
{
    // 가속도 기반 Pitch (deg)
    float denom = sqrtf(ay*ay + az*az);
    if (denom < 1e-6f) denom = 1e-6f;   // 0 나눗셈 방지
    float pitch_acc_deg = atan2f(-ax, denom) * (180.0f / (float)M_PI) + PITCH_ACC_OFFSET;

    // 자이로 적분 예측 (deg)
    static float pitch_est_deg = 0.0f;
    float pitch_gyr_pred = pitch_est_deg + gy * Ts;

    // 상보 필터 결합
    pitch_est_deg = alpha * pitch_gyr_pred + (1.0f - alpha) * pitch_acc_deg;

    return pitch_est_deg;
}

float BALANCE_UpdatePitch(float ax, float ay, float az, float gy)
{
	// 자이로(rad/s)
	float angle_gyr = gy * (float)M_PI / 180.0f;

    // 가속도 기반 피치(rad) — 롤 영향 제거
    float angle_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

    // 상보 필터
    float alpha = tau / (tau + Ts);

    pitch_rad = alpha * (pitch_rad + angle_gyr * Ts) + (1.0f - alpha) * angle_acc;

    return pitch_rad;
}

// --------------- 밸런스 관련 함수 ---------------

void Balance_Init(void)
{
  s_enabled = false;
  s_target_speed = 0.0f;
  s_pitch_deg = 0.0f;

  // ★ pidSetting(...) 대신
  PID_Init(&s_pid_angle, ANG_KP, ANG_KI, ANG_KD, ANG_ISUM, ANG_OUT);
  PID_Init(&s_pid_speed, SPD_KP, SPD_KI, SPD_KD, SPD_ISUM, SPD_OUT);

  // 필요 시 초깃값 리셋
  s_pid_angle.integral = 0.0f;
  s_pid_speed.integral = 0.0f;
  s_pid_angle.prev_error = 0.0f;
  s_pid_speed.prev_error = 0.0f;
}

void Balance_Enable(bool enable)
{
  if (!enable) {
    MOTOR_SetDuty_ID(MOTOR_LEFT , 0.0f);
    MOTOR_SetDuty_ID(MOTOR_RIGHT, 0.0f);
  } else {
    // ★ pid_t에는 output_sum이 없습니다 → integral로 리셋
    s_pid_angle.integral = 0.0f;
    s_pid_speed.integral = 0.0f;
    s_pid_angle.prev_error = 0.0f;
    s_pid_speed.prev_error = 0.0f;
  }
  s_enabled = enable;
}

bool Balance_IsEnabled(void){ return s_enabled; }

void Balance_SetTargetSpeed(float speed_ms)
{
  // 테스트 안전 범위: 약 ±1 m/s
  s_target_speed = clamp_float(speed_ms, -1.0f, 1.0f);
}

float Balance_GetCurrentAngle(void)
{
  return s_pitch_deg;
}

// 5ms 주기 호출(ENCODER_Update와 동일 ISR)
void Balance_Update(void)
{
  if (!s_enabled) return;

  float acc_g[3], gy_dps[3];
  MPU6050_GetAccelG(acc_g);
  MPU6050_GetGyroDPS(gy_dps);

  float gy_pitch  = GY_PITCH_SIGN * gy_dps[GY_PITCH_IDX];

  // ★ 상보필터로 pitch(deg) 추정 (이미 있는 함수 사용)
  s_pitch_deg = BALANCE_Angle_GetPitchCF(acc_g[0], acc_g[1], acc_g[2], gy_pitch);

  if (fabsf(s_pitch_deg) > MAX_TILT_ANGLE) {
    Balance_Enable(false);
    return;
  }

  // 속도 → m/s
  float rpmL = ENCODER_GetRPM_ID(MOTOR_LEFT );
  float rpmR = ENCODER_GetRPM_ID(MOTOR_RIGHT);
  float rpmAvg = 0.5f * (rpmL + rpmR);

  const float wheel_r = 0.034f;
  const float wheel_c = 2.0f * (float)M_PI * wheel_r;
  float speed_ms = (rpmAvg / 60.0f) * wheel_c;

  // outer(speed) PID: setpoint = s_target_speed
  s_pid_speed.setpoint = s_target_speed;
  float pitch_cmd_deg = PID_Compute(&s_pid_speed, speed_ms);  // deg 등가

  // inner(angle) PID: setpoint = pitch_cmd_deg
  float angle_meas_deg = deadband_filter(s_pitch_deg, DEADBAND_ANGLE);
  s_pid_angle.setpoint = pitch_cmd_deg;

  float duty = PID_Compute(&s_pid_angle, angle_meas_deg);
  duty = clamp_float(duty, -ANG_OUT, +ANG_OUT);

  MOTOR_SetDuty_ID(MOTOR_LEFT , duty);
  MOTOR_SetDuty_ID(MOTOR_RIGHT, duty);
}

// ---- 런타임 튠 ----
// 현재 제한값을 보존하려면 out/isum을 읽어 재초기화
static void reinit_angle_pid_(float kp, float ki, float kd)
{
  float isum = s_pid_angle.isum_max;     // (양/음 대칭이므로 +만 쓰면 충분)
  float out  = s_pid_angle.out_max;
  PID_Init(&s_pid_angle, kp, ki, kd, isum, out);
}
static void reinit_speed_pid_(float kp, float ki, float kd)
{
  float isum = s_pid_speed.isum_max;
  float out  = s_pid_speed.out_max;
  PID_Init(&s_pid_speed, kp, ki, kd, isum, out);
}

void Balance_SetAnglePID(float kp, float ki, float kd)
{
  reinit_angle_pid_(kp, ki, kd);
}
void Balance_SetSpeedPID(float kp, float ki, float kd)
{
  reinit_speed_pid_(kp, ki, kd);
}
