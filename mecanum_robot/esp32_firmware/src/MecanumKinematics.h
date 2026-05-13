/**
 * @file MecanumKinematics.h
 * @brief Động học thuận và nghịch cho bánh Mecanum
 * 
 * Bố cục bánh Mecanum (nhìn từ trên xuống):
 * 
 *     FL (/)=====(\) FR
 *        |         |
 *        |  ROBOT  |
 *        |         |
 *     RL (\)=====(/  RR
 * 
 * Ký hiệu:
 *   Vx  = vận tốc tiến/lùi (m/s) [+tiến, -lùi]
 *   Vy  = vận tốc sang ngang (m/s) [+phải, -trái]
 *   Vz  = vận tốc quay (rad/s) [+thuận chiều kim, -ngược]
 * 
 * Công thức động học nghịch (Inverse Kinematics):
 *   ω_FL = (Vx - Vy - (Lx+Ly)·Vz) / R
 *   ω_FR = (Vx + Vy + (Lx+Ly)·Vz) / R
 *   ω_RL = (Vx + Vy - (Lx+Ly)·Vz) / R
 *   ω_RR = (Vx - Vy + (Lx+Ly)·Vz) / R
 * 
 * Tham khảo: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
 */

#pragma once
#include <Arduino.h>
#include "config.h"

struct WheelSpeeds {
    double fl_rpm;  // Front-Left
    double fr_rpm;  // Front-Right
    double rl_rpm;  // Rear-Left
    double rr_rpm;  // Rear-Right
};

struct RobotVelocity {
    double vx;      // m/s - tiến/lùi
    double vy;      // m/s - sang ngang
    double vz;      // rad/s - quay
};

class MecanumKinematics {
public:
    MecanumKinematics(double wheel_radius = WHEEL_RADIUS_M,
                      double lx = WHEEL_BASE_X_M,
                      double ly = WHEEL_BASE_Y_M)
        : _r(wheel_radius), _lxy(lx + ly)
    {}

    /**
     * @brief Tính RPM từng bánh từ vận tốc xe (Inverse Kinematics)
     */
    WheelSpeeds inverseKinematics(double vx, double vy, double vz) {
        // Giới hạn vận tốc
        vx = constrain(vx, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        vy = constrain(vy, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

        double fl_rads = (vx - vy - _lxy * vz) / _r;
        double fr_rads = (vx + vy + _lxy * vz) / _r;
        double rl_rads = (vx + vy - _lxy * vz) / _r;
        double rr_rads = (vx - vy + _lxy * vz) / _r;

        // Chuẩn hóa nếu vượt quá MAX_RPM
        double max_rads = (MAX_RPM / 60.0) * 2.0 * PI;
        double scale = 1.0;
        scale = max(scale, abs(fl_rads) / max_rads);
        scale = max(scale, abs(fr_rads) / max_rads);
        scale = max(scale, abs(rl_rads) / max_rads);
        scale = max(scale, abs(rr_rads) / max_rads);

        if (scale > 1.0) {
            fl_rads /= scale;
            fr_rads /= scale;
            rl_rads /= scale;
            rr_rads /= scale;
        }

        // Chuyển rad/s → RPM
        double k = 60.0 / (2.0 * PI);
        return {
            fl_rads * k,
            fr_rads * k,
            rl_rads * k,
            rr_rads * k
        };
    }

    /**
     * @brief Tính vận tốc xe từ RPM từng bánh (Forward Kinematics)
     * Dùng để tính Odometry
     */
    RobotVelocity forwardKinematics(double fl_rpm, double fr_rpm,
                                     double rl_rpm, double rr_rpm) {
        // Chuyển RPM → rad/s
        double k = (2.0 * PI) / 60.0;
        double fl = fl_rpm * k;
        double fr = fr_rpm * k;
        double rl = rl_rpm * k;
        double rr = rr_rpm * k;

        return {
            (fl + fr + rl + rr) * _r / 4.0,
            (-fl + fr + rl - rr) * _r / 4.0,
            (-fl + fr - rl + rr) * _r / (4.0 * _lxy)
        };
    }

private:
    double _r;    // Bán kính bánh (m)
    double _lxy;  // Lx + Ly (m)
};
