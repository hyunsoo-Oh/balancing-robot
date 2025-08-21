# balancing-robot
JGB37-520 330RPM, MPU-6050을 활용한 밸런싱 로봇

## 📂 프로젝트 구조
```
ProjectRoot/
├── App/
│   ├── device/        # 외부 디바이스(센서 등) 드라이버 폴더
│   │   ├── motor_encoder.h
│   │   └── motor_encoder.c
│   └── ap.c           # 실제 동작을 위한 예제 코드
```

## CubeMX 설정

### MPU-6050 설정
```
Protocol : I2C  

Parameter Settings:
- Mode : Fast Mode (400kHz)
- Fast Mode Duty Cycle : Duty Cycle Tlow/Thigh = 2
- DMA : Enable
```

### JGB37-520 설정

#### 엔코더1용 - 32bit
```
Mode: Combined Channels > Encoder Mode

Parameter Settings:
- Encoder Mode: TI1 and TI2
- Counter Mode: Up
- Counter Period: 0xFFFFFFFF (32bit 최대값)
- Prescaler: 0
- Polarity: Rising Edge
```
#### 엔코더2용 - 32bit
```
Mode: Combined Channels > Encoder Mode

Parameter Settings:
- Encoder Mode: TI1 and TI2
- Counter Mode: Up
- Counter Period: 0xFFFFFFFF (32bit 최대값)
- Prescaler: 0
- Polarity: Rising Edge
```
#### 모터 PWM 제어용
```
Mode: 
- Channel1: PWM Generation CH1
- Channel2: PWM Generation CH2  
- Channel3: PWM Generation CH3
- Channel4: PWM Generation CH4

Parameter Settings:
- Prescaler: 899 (APB2 클럭이 180MHz일 때)
- Counter Mode: Up
- Counter Period: 999 (1kHz PWM 주파수)
- Pulse: 0 (초기 듀티사이클 0%)
- PWM Mode: PWM mode 1
```

#### L298N 제어를 위한 추가 GPIO 설정
```
GPIO Output pins:
- PB0: Motor1_DIR1 (IN1)
- PB1: Motor1_DIR2 (IN2)
- PB2: Motor2_DIR1 (IN3)  
- PB3: Motor2_DIR2 (IN4)

GPIO Mode: Output Push Pull
GPIO Pull-up/Pull-down: No pull-up and no pull-down
Maximum output speed: High
```

## 밸런싱 로봇 알고리즘

```
double input, output;
double setpoint = 180;
double Kp = 25.0, Ki = Ki = 80.0, Kd = 1.2;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
double fallLimit = 15;

