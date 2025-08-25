/*
 * mpu6050.c
 *
 *  Created on: Aug 11, 2025
 *      Author: jweli
 */

#include "mpu6050.h"
#include "usart.h"
#include "i2c.h"
/*
 *  대략적인 설정 값 (MPU-6050 기준)
 *  dlpf_cfg     |  Bandwidth         |   gyro_fs_sel       |   accel_fs_sel     |  smplrt_div 의미
 *  ------------ | ------------------ | ------------------- | ------------------ | -------------------------
 *  0x00         | 260Hz (delay: 0ms) | 0x00 : ±250°/s      | 0x00 : ±2g         | SampleRate = 1000 / (1 + div)
 *  0x01         | 184Hz (2ms)        | 0x08 : ±500°/s      | 0x08 : ±4g         | DLPF 사용 시 1kHz
 *  0x02         | 94Hz  (3ms)        | 0x10 : ±1000°/s     | 0x10 : ±8g         | DLPF 미사용 시 8kHz
 *  0x03         | 44Hz  (4.9ms)      | 0x18 : ±2000°/s     | 0x18 : ±16g        |
 *  0x04         | 21Hz  (8.5ms)
 *  0x05         | 10Hz  (13.8ms)
 *  0x06         | 5Hz   (19.0ms)
 *  0x07         | Reserved
 *
 *  dlpf_cfg의 Bandwidth와 smplrt_div의 주기를 잘 맞추는 것이 센서 노이즈 최소화, 제어 안정성, 데이터 낭비 방지에 매우 중요
 */

#define I2C_BUS		&hi2c1
#define UART_BUS	&huart1

static MPU6050_Config_t cfg = {
	.dlpf_cfg 		= 0x01,
	.gyro_fs_sel 	= 0x08,
	.accel_fs_sel 	= 0x00,
	.smplrt_div 	= 0x01,
};

static uint8_t buf[14];

static int16_t accel_offset[3] 	= {0, 0, 0};  // X, Y, Z 가속도 오프셋
static int16_t gyro_offset[3] 	= {0, 0, 0};  // X, Y, Z 자이로 오프셋

static HAL_StatusTypeDef I2C_Write(uint8_t addr, uint8_t reg, uint8_t data)
{
	uint8_t buff = data;
	return HAL_I2C_Mem_Write(I2C_BUS, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &buff, 1, 100);
}

static HAL_StatusTypeDef I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(I2C_BUS, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100);
}

static HAL_StatusTypeDef MPU6050_Write(uint8_t reg, uint8_t data)
{
	return I2C_Write(MPU6050_ADDRESS, reg, data);
}
static HAL_StatusTypeDef MPU6050_Read(uint8_t reg, uint8_t *data, uint8_t length)
{
	return I2C_Read(MPU6050_ADDRESS, reg, data, length);
}
HAL_StatusTypeDef MPU6050_Read_DMA(uint8_t reg, uint8_t *data, uint8_t length)
{
	HAL_I2C_Mem_Read_DMA(I2C_BUS, MPU6050_ADDRESS << 1, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buf, 14);
}

void MPU6050_Init(void)
{
	uint8_t who = 0;

	MPU6050_Write(MPU6050_PWR_MGMT_1, 0x00);	// Sleep Mode 해제
	HAL_Delay(100);

	MPU6050_Read(MPU6050_WHO_AM_I, &who, 1);

	if (who == 0x68)
		HAL_UART_Transmit(UART_BUS, "WHO_AM_I Read Success\r\n", sizeof("WHO_AM_I Read Success\r\n"), 100);
	else
		HAL_UART_Transmit(UART_BUS, "WHO_AM_I Read Failed\r\n", sizeof("WHO_AM_I Read Failed\r\n"), 100);

	MPU6050_Write(MPU6050_CONFIG, cfg.dlpf_cfg);
	MPU6050_Write(MPU6050_SMPLRT_DIV, cfg.smplrt_div);

	MPU6050_Write(MPU6050_GYRO_CONFIG, cfg.gyro_fs_sel);
	MPU6050_Write(MPU6050_ACCEL_CONFIG, cfg.accel_fs_sel);

	MPU6050_Calibrate();
	HAL_Delay(10);
}

void MPU6050_READ_RawAccel(int16_t ac_data[3])
{
    uint8_t buf[6];
    MPU6050_Read(MPU6050_ACCEL_XOUT_H, buf, 6);

    ac_data[0] = (int16_t)(buf[0] << 8 | buf[1]);
    ac_data[1] = (int16_t)(buf[2] << 8 | buf[3]);
    ac_data[2] = (int16_t)(buf[4] << 8 | buf[5]);
}
void MPU6050_READ_RawGyro(int16_t gy_data[3])
{
    uint8_t buf[6];
    MPU6050_Read(MPU6050_GYRO_XOUT_H, buf, 6);

    gy_data[0] = (int16_t)(buf[0] << 8 | buf[1]);
    gy_data[1] = (int16_t)(buf[2] << 8 | buf[3]);
    gy_data[2] = (int16_t)(buf[4] << 8 | buf[5]);
}
void MPU6050_READ_RawTemp(int16_t *te_data)
{
    uint8_t buf[2];
    MPU6050_Read(MPU6050_TEMP_OUT_H, buf, 2);

    *te_data = (int16_t)(buf[0] << 8 | buf[1]);
}

void MPU6050_READ_RawData_DMA(int16_t data[6])
{
	MPU6050_Read_DMA(MPU6050_ACCEL_XOUT_H, buf, 14);

	data[0] = (int16_t)(buf[0] << 8 | buf[1]);
	data[1] = (int16_t)(buf[2] << 8 | buf[3]);
	data[2] = (int16_t)(buf[4] << 8 | buf[5]);

	data[3] = (int16_t)(buf[8]  << 8 | buf[9]);
	data[4] = (int16_t)(buf[10] << 8 | buf[11]);
	data[5] = (int16_t)(buf[12] << 8 | buf[13]);
}

// 오프셋이 보정된 가속도 데이터 읽기
void MPU6050_ReadAccel(int16_t ac_data[3])
{
	MPU6050_READ_RawAccel(ac_data);

    // 오프셋 보정
    ac_data[0] -= accel_offset[0];
    ac_data[1] -= accel_offset[1];
    ac_data[2] -= accel_offset[2];
}

// 오프셋이 보정된 자이로 데이터 읽기
void MPU6050_ReadGyro(int16_t gy_data[3])
{
	MPU6050_READ_RawGyro(gy_data);

    // 오프셋 보정
    gy_data[0] -= gyro_offset[0];
    gy_data[1] -= gyro_offset[1];
    gy_data[2] -= gyro_offset[2];
}

void MPU6050_ReadData_DMA(int16_t data[6])
{
	MPU6050_READ_RawData_DMA(data);

    // 오프셋 보정
	data[0] -= accel_offset[0];
	data[1] -= accel_offset[1];
	data[2] -= accel_offset[2];

	data[3] -= gyro_offset[0];
	data[4] -= gyro_offset[1];
	data[5] -= gyro_offset[2];
}

// 실제 물리 단위로 변환하는 함수
void MPU6050_GetAccelG(float accel_g[3])
{
    int16_t raw_data[3];
    float lsb_per_g;

    MPU6050_ReadAccel(raw_data);

    // 현재 설정에 따른 LSB/g 값 계산
    switch (cfg.accel_fs_sel) {
        case 0x00: lsb_per_g = 16384.0f; break; // ±2g
        case 0x08: lsb_per_g = 8192.0f;  break; // ±4g
        case 0x10: lsb_per_g = 4096.0f;  break; // ±8g
        case 0x18: lsb_per_g = 2048.0f;  break; // ±16g
        default:   lsb_per_g = 4096.0f;  break;
    }

    accel_g[0] = (float)raw_data[0] / lsb_per_g;
    accel_g[1] = (float)raw_data[1] / lsb_per_g;
    accel_g[2] = (float)raw_data[2] / lsb_per_g;
}

void MPU6050_GetGyroDPS(float gyro_dps[3])
{
    int16_t raw_data[3];
    float lsb_per_dps;

    MPU6050_ReadGyro(raw_data);

    // 현재 설정에 따른 LSB/°/s 값 계산
    switch (cfg.gyro_fs_sel) {
        case 0x00: lsb_per_dps = 131.0f;  break; // ±250°/s
        case 0x08: lsb_per_dps = 65.5f;   break; // ±500°/s
        case 0x10: lsb_per_dps = 32.8f;   break; // ±1000°/s
        case 0x18: lsb_per_dps = 16.4f;   break; // ±2000°/s
        default:   lsb_per_dps = 32.8f;   break;
    }

    gyro_dps[0] = (float)raw_data[0] / lsb_per_dps;
    gyro_dps[1] = (float)raw_data[1] / lsb_per_dps;
    gyro_dps[2] = (float)raw_data[2] / lsb_per_dps;
}

float MPU6050_GetTempCelsius(void)
{
    int16_t raw_temp;
    MPU6050_READ_RawTemp(&raw_temp);

    // MPU-6050 온도 공식: Temperature = (TEMP_OUT / 340) + 36.53
    return (float)raw_temp / 340.0f + 36.53f;
}

void MPU6050_GetAccGyrData_DMA(float accel_g[3], float gyro_dps[3])
{
    int16_t raw_data[6];
    float lsb_per_g, lsb_per_dps;

	MPU6050_ReadData_DMA(raw_data);

    // 현재 설정에 따른 LSB/g 값 계산
    switch (cfg.accel_fs_sel) {
        case 0x00: lsb_per_g = 16384.0f; break; // ±2g
        case 0x08: lsb_per_g = 8192.0f;  break; // ±4g
        case 0x10: lsb_per_g = 4096.0f;  break; // ±8g
        case 0x18: lsb_per_g = 2048.0f;  break; // ±16g
        default:   lsb_per_g = 4096.0f;  break;
    }

    accel_g[0] = (float)raw_data[0] / lsb_per_g;
    accel_g[1] = (float)raw_data[1] / lsb_per_g;
    accel_g[2] = (float)raw_data[2] / lsb_per_g;

    // 현재 설정에 따른 LSB/°/s 값 계산
    switch (cfg.gyro_fs_sel) {
        case 0x00: lsb_per_dps = 131.0f;  break; // ±250°/s
        case 0x08: lsb_per_dps = 65.5f;   break; // ±500°/s
        case 0x10: lsb_per_dps = 32.8f;   break; // ±1000°/s
        case 0x18: lsb_per_dps = 16.4f;   break; // ±2000°/s
        default:   lsb_per_dps = 32.8f;   break;
    }

    gyro_dps[0] = (float)raw_data[3] / lsb_per_dps;
    gyro_dps[1] = (float)raw_data[4] / lsb_per_dps;
    gyro_dps[2] = (float)raw_data[5] / lsb_per_dps;
}

void MPU6050_Calibrate(void)
{
	switch (cfg.accel_fs_sel)
	{
		case 0x00:
			accel_offset[0] = -453;
			accel_offset[1] = 226;
			accel_offset[2] = -106;
			break;
		case 0x08:
			accel_offset[0] = -226;
			accel_offset[1] = -6;
			accel_offset[2] = -702;
			break;
		case 0x10:
			accel_offset[0] = 111;
			accel_offset[1] = -50;
			accel_offset[2] = 49;
			break;
		case 0x18:
			accel_offset[0] = 54;
			accel_offset[1] = -21;
			accel_offset[2] = -167;
			break;
	}
	switch (cfg.gyro_fs_sel)
	{
		case 0x00:
			gyro_offset[0] = -443;
			gyro_offset[1] = 154;
			gyro_offset[2] = 28;
			break;
		case 0x08:
			gyro_offset[0] = -213;
			gyro_offset[1] = 51;
			gyro_offset[2] = 25;
			break;
		case 0x10:
			gyro_offset[0] = -55;
			gyro_offset[1] = 25;
			gyro_offset[2] = -25;
			break;
		case 0x18:
			gyro_offset[0] = -56;
			gyro_offset[1] = 23;
			gyro_offset[2] = -26;
			break;
	}
}

void MPU6050_OffsetMeasure(void)
{
    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};
    int16_t accel_data[3];
    int16_t gyro_data[3];
    const int samples = 1000;  // 캘리브레이션 샘플 수

    int16_t expected_z = 0;
    switch (cfg.accel_fs_sel) {
        case 0x00: expected_z = 16384; break; // ±2g
        case 0x08: expected_z = 8192;  break; // ±4g
        case 0x10: expected_z = 4096;  break; // ±8g
        case 0x18: expected_z = 2048;  break; // ±16g
        default:   expected_z = 4096;  break;
    }

    // 안정화를 위한 대기
    HAL_Delay(1000);

    // 여러 샘플을 읽어서 평균값 계산
    for (int i = 0; i < samples; i++)
    {
    	MPU6050_READ_RawAccel(accel_data);
    	MPU6050_READ_RawGyro(gyro_data);

        // 가속도 데이터 누적
        accel_sum[0] += accel_data[0];
        accel_sum[1] += accel_data[1];
        accel_sum[2] += accel_data[2];

        // 자이로 데이터 누적
        gyro_sum[0] += gyro_data[0];
        gyro_sum[1] += gyro_data[1];
        gyro_sum[2] += gyro_data[2];

        HAL_Delay(5);  // 샘플 간격
    }
    // 평균값 계산하여 오프셋으로 설정
    accel_offset[0] = accel_sum[0] / samples;
    accel_offset[1] = accel_sum[1] / samples;
    accel_offset[2] = accel_sum[2] / samples - expected_z; // Z축은 중력(1g) 보정

    gyro_offset[0] = gyro_sum[0] / samples;
    gyro_offset[1] = gyro_sum[1] / samples;
    gyro_offset[2] = gyro_sum[2] / samples;
}
