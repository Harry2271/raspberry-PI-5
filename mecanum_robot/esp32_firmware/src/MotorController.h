/**
 * @file MotorController.h
 * @brief Điều khiển PID vận tốc cho một motor có encoder
 * 
 * Sử dụng thư viện ESP32Encoder để đọc xung Hall encoder
 * và PID library để điều chỉnh vận tốc.
 * 
 * Vòng điều khiển:
 *   Setpoint (RPM) → PID → PWM → Motor → Encoder → RPM thực
 */

#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include "BTS7960Motor.h"
#include "config.h"

class MotorController {
public:
    MotorController(BTS7960Motor& driver, uint8_t enc_a, uint8_t enc_b, bool invert = false)
        : _driver(driver), _enc_a(enc_a), _enc_b(enc_b), _invert(invert),
          _setpoint(0.0), _input(0.0), _output(0.0),
          _pid(&_input, &_output, &_setpoint, PID_KP, PID_KI, PID_KD, DIRECT),
          _lastCount(0), _lastTime(0), _currentRPM(0.0)
    {}

    void begin() {
        _driver.begin();

        // Cấu hình ESP32Encoder (hỗ trợ đọc cả 2 pha A, B)
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        _encoder.attachFullQuad(_enc_a, _enc_b);
        _encoder.clearCount();

        // Cấu hình PID
        _pid.SetMode(AUTOMATIC);
        _pid.SetSampleTime(PID_SAMPLE_MS);
        _pid.SetOutputLimits(-PWM_MAX, PWM_MAX);

        _lastTime = millis();
    }

    /**
     * @brief Đặt vận tốc mục tiêu
     * @param rpm Vận tốc (RPM), âm = quay ngược
     */
    void setTargetRPM(double rpm) {
        rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
        _setpoint = _invert ? -rpm : rpm;
    }

    /**
     * @brief Gọi trong loop() để cập nhật PID và xuất PWM
     */
    void update() {
        unsigned long now = millis();
        unsigned long dt = now - _lastTime;

        if (dt >= PID_SAMPLE_MS) {
            // Tính RPM từ encoder
            long currentCount = _encoder.getCount();
            long delta = currentCount - _lastCount;
            
            // RPM = (xung / PPR_full_quad) / thời_gian(phút)
            // PPR full quad = 4 * PPR_vật_lý
            _currentRPM = (double)delta / (4.0 * ENCODER_PPR) / (dt / 60000.0);
            
            _lastCount = currentCount;
            _lastTime = now;
            _input = _currentRPM;

            // Tính PID và ra lệnh cho driver
            _pid.Compute();
            _driver.setSpeed((int16_t)_output);
        }
    }

    void stop() {
        _setpoint = 0.0;
        _driver.stop();
        _pid.SetMode(MANUAL);
        _output = 0;
        _pid.SetMode(AUTOMATIC);
    }

    double getCurrentRPM() const { return _currentRPM; }
    long getEncoderCount() const { return _encoder.getCount(); }
    void resetEncoder() { _encoder.clearCount(); _lastCount = 0; }

private:
    BTS7960Motor& _driver;
    uint8_t _enc_a, _enc_b;
    bool _invert;

    ESP32Encoder _encoder;

    double _setpoint, _input, _output;
    PID _pid;

    long _lastCount;
    unsigned long _lastTime;
    double _currentRPM;
};
