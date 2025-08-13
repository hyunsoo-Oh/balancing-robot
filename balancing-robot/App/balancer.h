/*
 * balancer.h
 *
 *  Created on: Aug 13, 2025
 *      Author: USER
 */

#ifndef BALANCER_H_
#define BALANCER_H_

#include "def.h"

// 상보 필터 구조체
typedef struct {
    float alpha;        // 가속도/자이로 가중치 (0.0~1.0)
    float angle;        // 현재 추정 각도 [deg]
    float bias;         // 자이로 바이어스 [deg/s]
    float dt;           // 샘플링 주기 [s]
} complementary_filter_t;

// 칼만 필터 구조체 (선택적 사용)
typedef struct {
    float angle;        // 추정 각도 [deg]
    float bias;         // 자이로 바이어스 [deg/s]
    float P[2][2];      // 오차 공분산 행렬
    float Q_angle;      // 프로세스 노이즈 (각도)
    float Q_bias;       // 프로세스 노이즈 (바이어스)
    float R_measure;    // 측정 노이즈
    float dt;           // 샘플링 주기 [s]
} kalman_filter_t;



#endif /* BALANCER_H_ */
