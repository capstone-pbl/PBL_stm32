# 🚛 무인 자율주행 물류로봇 (PBL)

> STM32F411 펌웨어 레포지토리 | MCU: Nucleo F411RE + ESP32-S3

## 프로젝트 개요

Blynk 앱으로 명령을 내리면 UWB 실시간 측위를 기반으로 지정 좌표까지 자율 이동하고, 도착 후 리프터와 푸셔 액추에이터로 화물을 처리한 뒤 복귀하는 **무인 자율주행 물류로봇**이다.

## 시스템 아키텍처

```
[Blynk 앱]
    ↓ WiFi
[ESP32-S3]
  UWB 앵커 삼각측위 (BU01, 앵커 3~4개 + 태그 1개)
  측위 결과 "cx,cy\n" UART 송신
    ↓ UART6 (9600 bps)
[STM32F411 Nucleo]
  Pure Pursuit 경로추종 (상위 제어기)
  PID 속도 제어 (하위 제어기)
  리프터 위치 제어
    ↓ PWM
[IBT-2 모터드라이버]
    ↓
[RB-35GM 엔코더 모터 × 2]

[OpenMV H7 Plus] → QR코드 인식 / 장애물 감지
[LM4075OE 리프트] + [LM4075-F 푸셔] → 화물 처리
```

## 하드웨어 스펙

| 항목 | 사양 |
|------|------|
| MCU | STM32F411RE (Nucleo-64) |
| 보조 MCU | ESP32-S3 |
| 모터 | RB-35GM (12V, 감속비 1/180, 26PPR) |
| 모터드라이버 | IBT-2 |
| 위치 측위 | BU01 UWB 모듈 (앵커 3~4 + 태그 1) |
| 카메라 | OpenMV H7 Plus (QR코드/장애물) |
| 액추에이터 | LM4075OE 리프트(엔코더 내장) + LM4075-F 푸셔 |
| 배터리 | 12V 5Ah |

## 핀맵

| 타이머/핀 | 용도 |
|-----------|------|
| TIM1 CH1~4 (PA8~PA11) | 바퀴 PWM 출력 (4채널) |
| TIM4 CH1~2 (PB6/PB7) | 좌 바퀴 엔코더 |
| TIM3 CH1~2 (PB4/PB5) | 우 바퀴 엔코더 |
| TIM2 CH1~2 (PA0/PA1) | 리프트 엔코더 |
| USART6 (PC6/PC7) | ESP32 UART 통신 (9600 bps) |
| GPIO PC0~3 | 액추에이터 방향 제어 |

## STM32 펌웨어 주요 기능

### 1. Pure Pursuit 경로추종 알고리즘 (상위 제어기)
- UWB로부터 실시간 좌표(cx, cy) 수신
- 목표 좌표(tx, ty)까지의 거리(dist)와 heading 오차(alpha) 계산
- **alpha** = atan2(cy-ty, tx-cx) - θ
- 거리·각도 기반 비례 제어로 좌/우 목표 속도 산출

```cpp
// navigation.cpp
float v     = kp_pos * dist;        // 거리에 비례한 선속도
float omega = kp_ang * alpha;       // 각도 오차에 비례한 각속도
left_v  = v - omega * L / 2.0f;
right_v = v + omega * L / 2.0f;
```

### 2. PID 속도 제어 (하위 제어기)
- 바퀴 엔코더로 실측 속도(m/s) 계산 후 Pure Pursuit 목표속도 추종
- 좌/우 독립 PID: PL={Kp:19, Ki:37, Kd:0.18}, PR={Kp:21, Ki:40, Kd:0.18}
- I항 windup 방지 (±0.135 m/s 클램핑)
- 쿼드라처 엔코더: 26 PPR × 180(감속비) × 4 = **18,720 counts/rev**

### 3. 바퀴 오도메트리
- 엔코더 delta로 이동거리(delta_s)·회전각(delta_theta) 계산
- UWB 위치 점프 감지 시 theta 보정 (이동거리 0.15m 이상)

```cpp
delta_s     = (delta_encR + delta_encL) / 2.0f;
delta_theta = (delta_encR - delta_encL) / L;      // L = 바퀴간격 0.39m
```

### 4. 리프터 액추에이터 위치 제어
- LM4075OE 내장 엔코더 기반 bang-bang 위치 제어
- 변환 계수: **411.4 counts/mm**
- DeadZone 1mm 이내 도달 시 정지

### 5. UART 통신 (ESP32 ↔ STM32)
- ESP32가 UWB 삼각측위 후 `"cx,cy\n"` 형식으로 UART 전송
- Singleton 패턴 `uart_comm` 클래스가 인터럽트 기반으로 수신·파싱

## 소프트웨어 아키텍처 (C++ OOP)

```
Core/
├── Inc/
│   ├── motor.hpp              # PWM 모터 제어
│   ├── navigation.hpp         # Pure Pursuit + PID + 오도메트리
│   ├── actuator.hpp           # 리프터/푸셔 위치 제어
│   └── uart_communication.hpp # ESP32 UART 수신 (Singleton)
└── Src/
    ├── motor.cpp
    ├── navigation.cpp
    ├── actuator.cpp
    ├── uart_communication.cpp
    └── main.cpp               # 객체 생성, 초기화, 메인 루프
```

### 클래스 설계

| 클래스 | 역할 |
|--------|------|
| `run_motor` | TIM1 PWM duty 제어, 비상정지 |
| `Pure_pursuit` | 경로추종 계산, 오도메트리, PID 속도제어기 |
| `ACT` | 리프터 위치 bang-bang 제어, 푸셔 방향 제어 |
| `uart_comm` | ESP32 UART 인터럽트 수신, Singleton 패턴 |

## 통신 흐름

```
Blynk 앱 → (WiFi) → ESP32
ESP32: UWB 삼각측위 → "cx,cy\n" UART 전송
STM32: sscanf 파싱 → Pure Pursuit 입력
STM32 → PWM → IBT-2 → 모터
STM32 → GPIO → 액추에이터 드라이버
```

## 담당 역할 (6인 팀)

- **STM32 아키텍처 설계**: C++ 클래스 구조 설계, 전체 펌웨어 뼈대 구성
- **모터 제어** (`motor.hpp/cpp`): PWM duty 제어, TIM1 4채널 설정
- **UWB UART 연동** (`uart_communication.hpp/cpp`): ESP32-STM32 인터럽트 기반 통신, Singleton 패턴 구현
- `actuator.hpp/cpp`: 팀원 담당
