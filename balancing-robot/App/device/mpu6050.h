/*
 * mpu6050.h
 *
 *  Created on: Aug 11, 2025
 *      Author: jweli
 */

#ifndef DEVICE_MPU6050_H_
#define DEVICE_MPU6050_H_

#include "def.h"

// -----------------------------------------------------------------------------
// 기본 정보
// -----------------------------------------------------------------------------
#define MPU6050_ADDRESS            0x68  	   // AD0 == LOW ? 0x68 : 0x69
#define MPU6050_WHO_AM_I           0x75        // WHO_AM_I: 0x68 expected

// -----------------------------------------------------------------------------
// 전원 제어 (Power Management)
// -----------------------------------------------------------------------------
#define MPU6050_PWR_MGMT_1         0x6B        // Sleep mode, clock source 선택
#define MPU6050_PWR_MGMT_2         0x6C        // 센서별 standby 설정

// -----------------------------------------------------------------------------
// 센서 리셋 제어 (Signal Path Reset & User Control)
// -----------------------------------------------------------------------------
#define MPU6050_SIGNAL_PATH_RESET  0x68        // TEMP_RESET, ACCEL_RESET, GYRO_RESET
#define MPU6050_USER_CTRL          0x6A        // FIFO/I2C master/reset 제어

// -----------------------------------------------------------------------------
// 센서 설정 (Configuration Registers)
// -----------------------------------------------------------------------------
#define MPU6050_SMPLRT_DIV         0x19        // 샘플링 분주 설정
#define MPU6050_CONFIG             0x1A        // DLPF 설정 등
#define MPU6050_GYRO_CONFIG        0x1B        // Gyro Full Scale 설정
#define MPU6050_ACCEL_CONFIG       0x1C        // Accel Full Scale 설정

// -----------------------------------------------------------------------------
// 센서 출력 데이터 (Sensor Output Registers)
// -----------------------------------------------------------------------------
#define MPU6050_ACCEL_XOUT_H       0x3B
#define MPU6050_ACCEL_XOUT_L       0x3C
#define MPU6050_ACCEL_YOUT_H       0x3D
#define MPU6050_ACCEL_YOUT_L       0x3E
#define MPU6050_ACCEL_ZOUT_H       0x3F
#define MPU6050_ACCEL_ZOUT_L       0x40

#define MPU6050_TEMP_OUT_H         0x41
#define MPU6050_TEMP_OUT_L         0x42

#define MPU6050_GYRO_XOUT_H        0x43
#define MPU6050_GYRO_XOUT_L        0x44
#define MPU6050_GYRO_YOUT_H        0x45
#define MPU6050_GYRO_YOUT_L        0x46
#define MPU6050_GYRO_ZOUT_H        0x47
#define MPU6050_GYRO_ZOUT_L        0x48

// -----------------------------------------------------------------------------
// I2C 마스터 제어 (Master Mode to read external sensors)
// -----------------------------------------------------------------------------
#define MPU6050_I2C_MST_CTRL       0x24  // I2C 마스터 모드 설정 (클럭 속도, 멀티 마스터, 슬레이브 레지스터 지연 설정 등)

// I2C Slave 0 (외부 장치와의 인터페이스 설정)
#define MPU6050_I2C_SLV0_ADDR      0x25  // I2C 슬레이브 0 주소 (Read/Write 비트 포함)
#define MPU6050_I2C_SLV0_REG       0x26  // 슬레이브 장치에서 읽거나 쓸 레지스터 주소
#define MPU6050_I2C_SLV0_CTRL      0x27  // 슬레이브 0 제어: 읽기 길이, enable, 반복 여부 등 설정
#define MPU6050_I2C_SLV0_DO        0x63  // 마스터가 슬레이브로 전송할 데이터(Byte 1)

// I2C Master Delay Control
#define MPU6050_I2C_MST_DELAY_CTRL 0x67  // 각 I2C 슬레이브에 대한 전송 지연 삽입 여부 설정 (Slave0~3용 delay enable 비트)

// -----------------------------------------------------------------------------
// 외부 센서 데이터 버퍼 (External Sensor Data Registers)
// -----------------------------------------------------------------------------
#define MPU6050_EXT_SENS_DATA_0    0x49
#define MPU6050_EXT_SENS_DATA_1    0x4A
#define MPU6050_EXT_SENS_DATA_2    0x4B

// -----------------------------------------------------------------------------
// 인터럽트 제어 (Interrupt Control)
// -----------------------------------------------------------------------------
#define MPU6050_INT_PIN_CFG        0x37
#define MPU6050_INT_ENABLE         0x38
#define MPU6050_INT_STATUS         0x3A

// -----------------------------------------------------------------------------
// FIFO 제어 (FIFO Control Registers)
// -----------------------------------------------------------------------------
#define MPU6050_FIFO_EN            0x23
#define MPU6050_FIFO_COUNTH        0x72
#define MPU6050_FIFO_COUNTL        0x73
#define MPU6050_FIFO_R_W           0x74

typedef struct {
    uint8_t dlpf_cfg;       // CONFIG 레지스터 (0x1A)
    uint8_t gyro_fs_sel;    // GYRO_CONFIG 레지스터 (0x1B)
    uint8_t accel_fs_sel;   // ACCEL_CONFIG 레지스터 (0x1C)
    uint8_t smplrt_div;     // SMPLRT_DIV (0x19)
} MPU6050_Config_t;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
} MPU6050_Data_t;

// 초기화
void MPU6050_Init();

// 데이터 읽기
void MPU6050_READ_RawAccel(int16_t ac_data[3]);
void MPU6050_READ_RawGyro(int16_t gy_data[3]);
void MPU6050_READ_RawTemp(int16_t *te_data);

HAL_StatusTypeDef MPU6050_Read_DMA(uint8_t reg, uint8_t *data, uint8_t length);
void MPU6050_READ_RawData_DMA(int16_t data[6]);

// 오프셋 보정된 데이터 읽기 함수
void MPU6050_ReadAccel(int16_t ac_data[3]);    // 오프셋 보정된 가속도
void MPU6050_ReadGyro(int16_t gy_data[3]);     // 오프셋 보정된 자이로

void MPU6050_ReadData_DMA(int16_t data[6]);

// 오차 제거
void MPU6050_Calibrate(void);
void MPU6050_OffsetMeasure(void);

// 실제 물리 단위로 변환하는 함수
void  MPU6050_GetAccelG(float accel_g[3]);     // 가속도를 g 단위로
void  MPU6050_GetGyroDPS(float gyro_dps[3]);   // 자이로를 °/s 단위로
float MPU6050_GetTempCelsius(void);   		   // 온도를 섭씨로

void MPU6050_GetAccGyrData_DMA(float accel_g[3], float gyro_dps[3]);

#endif /* DEVICE_MPU6050_H_ */
