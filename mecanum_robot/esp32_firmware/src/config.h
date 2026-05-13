/**
 * @file config.h
 * @brief Cấu hình toàn cục cho xe tự hành Mecanum
 * 
 * Kit: WeAct ESP32-S3-A N16R8 (16MB Flash, 8MB OPI PSRAM)
 * Driver: 4x BTS7960 (H-Bridge)
 * Motor: 4x JGB37-520 với Hall Encoder (333 RPM)
 * 
 * Sơ đồ phân bổ GPIO:
 * ┌─────────────────────────────────────────────────┐
 * │  MOTOR FL (Front-Left)                          │
 * │    BTS7960: R_EN=GPIO1, L_EN=GPIO2             │
 * │             R_PWM=GPIO3, L_PWM=GPIO4           │
 * │    Encoder:  A=GPIO5,   B=GPIO6                │
 * ├─────────────────────────────────────────────────┤
 * │  MOTOR FR (Front-Right)                         │
 * │    BTS7960: R_EN=GPIO7, L_EN=GPIO8             │
 * │             R_PWM=GPIO9, L_PWM=GPIO10          │
 * │    Encoder:  A=GPIO11,  B=GPIO12               │
 * ├─────────────────────────────────────────────────┤
 * │  MOTOR RL (Rear-Left)                           │
 * │    BTS7960: R_EN=GPIO13, L_EN=GPIO14           │
 * │             R_PWM=GPIO15, L_PWM=GPIO16         │
 * │    Encoder:  A=GPIO17,  B=GPIO18               │
 * ├─────────────────────────────────────────────────┤
 * │  MOTOR RR (Rear-Right)                          │
 * │    BTS7960: R_EN=GPIO35, L_EN=GPIO36           │
 * │             R_PWM=GPIO37, L_PWM=GPIO38         │
 * │    Encoder:  A=GPIO39,  B=GPIO40               │
 * └─────────────────────────────────────────────────┘
 * 
 * ⚠ LƯU Ý ĐẤU NỐI BTS7960:
 *   - R_EN và L_EN nối với 3.3V qua chân GPIO (HIGH = Enable)
 *   - VCC Motor: 12V riêng, GND chung với ESP32
 *   - R_PWM và L_PWM: tín hiệu PWM 0-3.3V từ ESP32
 */

#pragma once
#include <Arduino.h>

// ======================================================
// WiFi Configuration
// ======================================================
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define UDP_PORT        8888          // Port lắng nghe lệnh từ Pi5
#define UDP_FEEDBACK_PORT 8889        // Port gửi feedback về Pi5
#define HEARTBEAT_TIMEOUT_MS 500      // ms không nhận lệnh -> dừng xe

// ======================================================
// BTS7960 GPIO Pins
// ======================================================

// Motor FL (Front-Left) - Bánh trước trái
#define FL_R_EN   1
#define FL_L_EN   2
#define FL_R_PWM  3
#define FL_L_PWM  4

// Motor FR (Front-Right) - Bánh trước phải
#define FR_R_EN   7
#define FR_L_EN   8
#define FR_R_PWM  9
#define FR_L_PWM  10

// Motor RL (Rear-Left) - Bánh sau trái
#define RL_R_EN   13
#define RL_L_EN   14
#define RL_R_PWM  15
#define RL_L_PWM  16

// Motor RR (Rear-Right) - Bánh sau phải
#define RR_R_EN   35
#define RR_L_EN   36
#define RR_R_PWM  37
#define RR_L_PWM  38

// ======================================================
// Encoder GPIO Pins
// ======================================================
#define FL_ENC_A  5
#define FL_ENC_B  6

#define FR_ENC_A  11
#define FR_ENC_B  12

#define RL_ENC_A  17
#define RL_ENC_B  18

#define RR_ENC_A  39
#define RR_ENC_B  40

// ======================================================
// PWM Configuration (ESP32-S3 LEDC)
// ======================================================
#define PWM_FREQ        20000   // 20 kHz (trên ngưỡng nghe thấy)
#define PWM_RESOLUTION  8       // 8-bit: 0-255
#define PWM_MAX         255

// LEDC Channel Assignment (8 kênh)
#define FL_R_CH  0
#define FL_L_CH  1
#define FR_R_CH  2
#define FR_L_CH  3
#define RL_R_CH  4
#define RL_L_CH  5
#define RR_R_CH  6
#define RR_L_CH  7

// ======================================================
// Motor & Kinematic Parameters
// ======================================================
#define ENCODER_PPR         333     // Pulses Per Revolution của JGB37-520
#define GEAR_RATIO          1.0     // Tỷ số truyền (đã tính vào PPR)
#define WHEEL_RADIUS_M      0.048   // Bán kính bánh Mecanum (m) - điều chỉnh theo thực tế
#define WHEEL_BASE_X_M      0.150   // Khoảng cách tâm bánh theo trục X (m)
#define WHEEL_BASE_Y_M      0.150   // Khoảng cách tâm bánh theo trục Y (m)
#define MAX_RPM             333.0   // RPM tối đa của motor
#define MAX_WHEEL_SPEED     ((MAX_RPM / 60.0) * 2.0 * PI * WHEEL_RADIUS_M) // m/s

// ======================================================
// PID Parameters (Cần tinh chỉnh thực nghiệm)
// ======================================================
#define PID_KP     2.5
#define PID_KI     0.8
#define PID_KD     0.05
#define PID_SAMPLE_MS  20   // 50Hz

// ======================================================
// Odometry
// ======================================================
#define ODOM_PUBLISH_MS  50    // Gửi odometry mỗi 50ms (20Hz)
