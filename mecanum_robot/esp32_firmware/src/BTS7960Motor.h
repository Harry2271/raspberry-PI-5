/**
 * @file BTS7960Motor.h
 * @brief Driver điều khiển BTS7960 H-Bridge
 * 
 * BTS7960 là IC H-Bridge công suất cao (43A peak).
 * Điều khiển bằng 2 tín hiệu PWM:
 *   - R_PWM: quay thuận (Forward)
 *   - L_PWM: quay nghịch (Reverse)
 * 
 * Sơ đồ logic:
 *   R_PWM  | L_PWM | Trạng thái
 *   -------|-------|----------
 *   PWM    |  0    | Quay thuận
 *   0      | PWM   | Quay nghịch
 *   0      |  0    | Dừng (Coast)
 *   PWM    | PWM   | Phanh (Brake) - TRÁNH dùng
 */

#pragma once
#include <Arduino.h>
#include "config.h"

class BTS7960Motor {
public:
    BTS7960Motor(uint8_t r_en, uint8_t l_en, 
                 uint8_t r_pwm, uint8_t l_pwm,
                 uint8_t r_channel, uint8_t l_channel)
        : _r_en(r_en), _l_en(l_en),
          _r_pwm(r_pwm), _l_pwm(l_pwm),
          _r_ch(r_channel), _l_ch(l_channel)
    {}

    void begin() {
        // Cấu hình chân Enable
        pinMode(_r_en, OUTPUT);
        pinMode(_l_en, OUTPUT);
        digitalWrite(_r_en, HIGH);  // Enable
        digitalWrite(_l_en, HIGH);  // Enable

        // Cấu hình LEDC (ESP32 PWM)
        ledcSetup(_r_ch, PWM_FREQ, PWM_RESOLUTION);
        ledcSetup(_l_ch, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(_r_pwm, _r_ch);
        ledcAttachPin(_l_pwm, _l_ch);

        stop();
    }

    /**
     * @brief Đặt tốc độ motor
     * @param speed Giá trị -255 đến +255
     *              Dương = quay thuận, Âm = quay nghịch
     */
    void setSpeed(int16_t speed) {
        speed = constrain(speed, -PWM_MAX, PWM_MAX);
        
        if (speed > 0) {
            // Quay thuận: R_PWM = speed, L_PWM = 0
            ledcWrite(_r_ch, (uint8_t)speed);
            ledcWrite(_l_ch, 0);
        } else if (speed < 0) {
            // Quay nghịch: R_PWM = 0, L_PWM = |speed|
            ledcWrite(_r_ch, 0);
            ledcWrite(_l_ch, (uint8_t)(-speed));
        } else {
            stop();
        }
    }

    /**
     * @brief Dừng motor (Coast - không hãm)
     */
    void stop() {
        ledcWrite(_r_ch, 0);
        ledcWrite(_l_ch, 0);
    }

    /**
     * @brief Kích hoạt/Vô hiệu hóa driver
     */
    void enable(bool en) {
        digitalWrite(_r_en, en ? HIGH : LOW);
        digitalWrite(_l_en, en ? HIGH : LOW);
    }

private:
    uint8_t _r_en, _l_en;
    uint8_t _r_pwm, _l_pwm;
    uint8_t _r_ch, _l_ch;
};
