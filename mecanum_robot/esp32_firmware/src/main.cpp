/**
 * @file main.cpp
 * @brief Firmware chính cho ESP32-S3 - Xe tự hành Mecanum
 * 
 * Luồng hoạt động:
 *   1. Khởi động WiFi, UDP Server
 *   2. Khởi tạo 4 BTS7960 + PID Controller
 *   3. Task0 (Core 0): Đọc lệnh UDP từ Pi5, cập nhật PID
 *   4. Task1 (Core 1): Tính Odometry, gửi feedback về Pi5
 * 
 * Giao thức UDP (JSON compact):
 *   Nhận (Pi5 → ESP32):
 *     {"vx":0.3,"vy":0.0,"vz":0.1,"seq":12}
 *   Gửi (ESP32 → Pi5):
 *     {"fl_rpm":100.2,"fr_rpm":100.1,"rl_rpm":99.8,"rr_rpm":100.0,
 *      "x":0.12,"y":0.05,"theta":0.01,"bat_v":11.8,"seq":12}
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>

#include "config.h"
#include "BTS7960Motor.h"
#include "MotorController.h"
#include "MecanumKinematics.h"
#include "Odometry.h"

// ======================================================
// Khai báo đối tượng
// ======================================================

// BTS7960 Drivers
BTS7960Motor driverFL(FL_R_EN, FL_L_EN, FL_R_PWM, FL_L_PWM, FL_R_CH, FL_L_CH);
BTS7960Motor driverFR(FR_R_EN, FR_L_EN, FR_R_PWM, FR_L_PWM, FR_R_CH, FR_L_CH);
BTS7960Motor driverRL(RL_R_EN, RL_L_EN, RL_R_PWM, RL_L_PWM, RL_R_CH, RL_L_CH);
BTS7960Motor driverRR(RR_R_EN, RR_L_EN, RR_R_PWM, RR_L_PWM, RR_R_CH, RR_L_CH);

// PID Motor Controllers
// FR và RR cần invert = true vì bánh phải quay ngược chiều so với trái
MotorController motorFL(driverFL, FL_ENC_A, FL_ENC_B, false);
MotorController motorFR(driverFR, FR_ENC_A, FR_ENC_B, true);
MotorController motorRL(driverRL, RL_ENC_A, RL_ENC_B, false);
MotorController motorRR(driverRR, RR_ENC_A, RR_ENC_B, true);

// Kinematics & Odometry
MecanumKinematics kinematics;
Odometry odometry;

// UDP
AsyncUDP udpServer;

// ======================================================
// Biến chia sẻ (thread-safe với SemaphoreHandle_t)
// ======================================================
SemaphoreHandle_t xMutex;

struct Command {
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    uint32_t seq = 0;
    unsigned long lastReceived = 0;
};
volatile Command currentCmd;
volatile bool cmdUpdated = false;

// IP của Raspberry Pi 5 (lấy từ UDP packet)
IPAddress pi5IP(0, 0, 0, 0);
bool pi5Known = false;

// ======================================================
// Hàm tiện ích
// ======================================================
void emergencyStop() {
    motorFL.stop();
    motorFR.stop();
    motorRL.stop();
    motorRR.stop();
    Serial.println("[SAFETY] Emergency Stop!");
}

void connectWiFi() {
    Serial.printf("[WiFi] Đang kết nối đến '%s'...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Kết nối thành công! IP: %s\n", 
                      WiFi.localIP().toString().c_str());
        Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
    } else {
        Serial.println("\n[WiFi] THẤT BẠI! Kiểm tra SSID/Password.");
    }
}

// ======================================================
// Xử lý gói tin UDP từ Pi5
// ======================================================
void onUDPPacket(AsyncUDPPacket& packet) {
    // Lưu IP của Pi5
    if (!pi5Known) {
        pi5IP = packet.remoteIP();
        pi5Known = true;
        Serial.printf("[UDP] Pi5 phát hiện tại: %s\n", pi5IP.toString().c_str());
    }

    // Parse JSON
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, 
                                               (const char*)packet.data(), 
                                               packet.length());
    if (err) {
        Serial.printf("[UDP] JSON lỗi: %s\n", err.c_str());
        return;
    }

    // Cập nhật lệnh (thread-safe)
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        currentCmd.vx           = doc["vx"] | 0.0;
        currentCmd.vy           = doc["vy"] | 0.0;
        currentCmd.vz           = doc["vz"] | 0.0;
        currentCmd.seq          = doc["seq"] | 0;
        currentCmd.lastReceived = millis();
        cmdUpdated = true;
        xSemaphoreGive(xMutex);
    }
}

// ======================================================
// Task điều khiển motor (Core 0)
// ======================================================
void taskMotorControl(void* param) {
    Serial.println("[Task0] Motor Control Task started on Core 0");
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        double vx = 0, vy = 0, vz = 0;
        bool timedOut = false;

        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            vx = currentCmd.vx;
            vy = currentCmd.vy;
            vz = currentCmd.vz;
            timedOut = (millis() - currentCmd.lastReceived) > HEARTBEAT_TIMEOUT_MS;
            xSemaphoreGive(xMutex);
        }

        if (timedOut) {
            emergencyStop();
        } else {
            // Tính RPM từng bánh
            WheelSpeeds ws = kinematics.inverseKinematics(vx, vy, vz);
            motorFL.setTargetRPM(ws.fl_rpm);
            motorFR.setTargetRPM(ws.fr_rpm);
            motorRL.setTargetRPM(ws.rl_rpm);
            motorRR.setTargetRPM(ws.rr_rpm);
        }

        // Cập nhật PID cho 4 motor
        motorFL.update();
        motorFR.update();
        motorRL.update();
        motorRR.update();

        // Chạy 50Hz
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// ======================================================
// Task Odometry & Telemetry (Core 1)
// ======================================================
void taskTelemetry(void* param) {
    Serial.println("[Task1] Telemetry Task started on Core 1");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    odometry.reset();

    for (;;) {
        // Đọc RPM thực tế
        float fl = (float)motorFL.getCurrentRPM();
        float fr = (float)motorFR.getCurrentRPM();
        float rl = (float)motorRL.getCurrentRPM();
        float rr = (float)motorRR.getCurrentRPM();

        // Tính vận tốc thực từ Forward Kinematics
        RobotVelocity vel = kinematics.forwardKinematics(fl, fr, rl, rr);
        odometry.update((float)vel.vx, (float)vel.vy, (float)vel.vz);
        Pose pose = odometry.getPose();

        // Gửi feedback về Pi5 qua UDP
        if (pi5Known && WiFi.status() == WL_CONNECTED) {
            StaticJsonDocument<256> doc;
            uint32_t seq = 0;
            if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                seq = currentCmd.seq;
                xSemaphoreGive(xMutex);
            }

            doc["fl"]    = fl;
            doc["fr"]    = fr;
            doc["rl"]    = rl;
            doc["rr"]    = rr;
            doc["x"]     = pose.x;
            doc["y"]     = pose.y;
            doc["th"]    = pose.theta;
            doc["seq"]   = seq;

            char buf[256];
            size_t len = serializeJson(doc, buf);
            udpServer.writeTo((uint8_t*)buf, len, pi5IP, UDP_FEEDBACK_PORT);
        }

        // Chạy 20Hz
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ODOM_PUBLISH_MS));
    }
}

// ======================================================
// setup() và loop()
// ======================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println(" Mecanum Robot - WeAct ESP32-S3-A N16R8");
    Serial.println("========================================");

    // Tạo mutex
    xMutex = xSemaphoreCreateMutex();

    // Khởi tạo motor & encoder
    Serial.println("[Init] Khởi tạo 4 Motor + PID...");
    motorFL.begin();
    motorFR.begin();
    motorRL.begin();
    motorRR.begin();
    Serial.println("[Init] OK!");

    // Kết nối WiFi
    connectWiFi();

    // Khởi động UDP Server
    if (udpServer.listen(UDP_PORT)) {
        udpServer.onPacket(onUDPPacket);
        Serial.printf("[UDP] Server lắng nghe tại port %d\n", UDP_PORT);
        Serial.printf("[UDP] Feedback port: %d\n", UDP_FEEDBACK_PORT);
    }

    // Khởi tạo timestamp để timeout
    currentCmd.lastReceived = millis();

    // Tạo FreeRTOS Tasks
    xTaskCreatePinnedToCore(taskMotorControl, "MotorCtrl", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(taskTelemetry,   "Telemetry", 4096, NULL, 1, NULL, 1);

    Serial.println("[Setup] Hoàn tất! Chờ lệnh từ Pi5...");
}

void loop() {
    // Kiểm tra WiFi và tự kết nối lại nếu mất kết nối
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Mất kết nối! Đang kết nối lại...");
        pi5Known = false;
        emergencyStop();
        connectWiFi();
    }
    delay(5000);
}
