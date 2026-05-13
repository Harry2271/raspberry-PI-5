/**
 * @file Odometry.h
 * @brief Tính toán Odometry cho xe Mecanum
 * 
 * Tích phân vận tốc theo thời gian để ước lượng vị trí (x, y, θ).
 */

#pragma once
#include <Arduino.h>
#include <math.h>

struct Pose {
    float x;      // vị trí X (m)
    float y;      // vị trí Y (m)
    float theta;  // góc quay (rad)
};

class Odometry {
public:
    Odometry() : _x(0.0f), _y(0.0f), _theta(0.0f), _lastUpdate(0) {}

    void reset() {
        _x = _y = _theta = 0.0f;
        _lastUpdate = millis();
    }

    /**
     * @brief Cập nhật pose từ vận tốc
     * @param vx  Vận tốc X (m/s)
     * @param vy  Vận tốc Y (m/s)
     * @param vz  Vận tốc quay (rad/s)
     */
    void update(float vx, float vy, float vz) {
        unsigned long now = millis();
        float dt = (now - _lastUpdate) / 1000.0f;
        _lastUpdate = now;

        if (dt <= 0.0f || dt > 0.5f) return;  // Bỏ qua dt bất thường

        // Chuyển từ frame robot → frame world
        float cos_t = cosf(_theta);
        float sin_t = sinf(_theta);

        _x     += (vx * cos_t - vy * sin_t) * dt;
        _y     += (vx * sin_t + vy * cos_t) * dt;
        _theta += vz * dt;

        // Chuẩn hóa góc [-pi, pi]
        while (_theta >  PI) _theta -= 2.0f * PI;
        while (_theta < -PI) _theta += 2.0f * PI;
    }

    Pose getPose() const { return {_x, _y, _theta}; }

private:
    float _x, _y, _theta;
    unsigned long _lastUpdate;
};
