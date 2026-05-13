# Mecanum Robot - Hướng dẫn đấu nối phần cứng

## Sơ đồ đấu nối tổng thể

```
                    ┌─────────────────────┐
                    │   NGUỒN 12V (LiPo)  │
                    └─────┬──────────┬────┘
                          │          │
                    ┌─────▼──┐  ┌───▼──────┐
                    │ BTS7960│  │ Buck 5V  │
                    │  x 4   │  │ (Pi5+ESP)│
                    └────┬───┘  └───┬──────┘
                         │          │
                   4 Motor    Raspberry Pi 5
                   JGB37-520       &
                   + Encoder   ESP32-S3
```

---

## Đấu nối BTS7960 với ESP32-S3

Mỗi BTS7960 cần **6 dây** từ ESP32:

| BTS7960 Pin | ESP32-S3 GPIO | Mô tả           |
|-------------|---------------|-----------------|
| R_EN        | GPIO 1        | Enable bên thuận|
| L_EN        | GPIO 2        | Enable bên ngược|
| R_PWM       | GPIO 3        | PWM bên thuận   |
| L_PWM       | GPIO 4        | PWM bên ngược   |
| VCC         | 3.3V          | Logic supply    |
| GND         | GND           | Ground chung    |
| B+ / B-     | 12V+ / 12V-   | Nguồn motor     |
| M+ / M-     | Motor+/Motor- | Dây ra motor    |

> ⚠️ **QUAN TRỌNG**: BTS7960 logic chạy **3.3V** - tương thích hoàn toàn với ESP32-S3.

---

## Phân bổ GPIO đầy đủ

### Motor Front-Left (FL) - Bánh trước trái
| Chức năng  | GPIO |
|------------|------|
| R_EN       | 1    |
| L_EN       | 2    |
| R_PWM      | 3    |
| L_PWM      | 4    |
| Encoder A  | 5    |
| Encoder B  | 6    |

### Motor Front-Right (FR) - Bánh trước phải
| Chức năng  | GPIO |
|------------|------|
| R_EN       | 7    |
| L_EN       | 8    |
| R_PWM      | 9    |
| L_PWM      | 10   |
| Encoder A  | 11   |
| Encoder B  | 12   |

### Motor Rear-Left (RL) - Bánh sau trái
| Chức năng  | GPIO |
|------------|------|
| R_EN       | 13   |
| L_EN       | 14   |
| R_PWM      | 15   |
| L_PWM      | 16   |
| Encoder A  | 17   |
| Encoder B  | 18   |

### Motor Rear-Right (RR) - Bánh sau phải
| Chức năng  | GPIO |
|------------|------|
| R_EN       | 35   |
| L_EN       | 36   |
| R_PWM      | 37   |
| L_PWM      | 38   |
| Encoder A  | 39   |
| Encoder B  | 40   |

---

## Đấu nối Encoder JGB37-520

Encoder Hall của JGB37-520 thường có **5 dây**:

| Màu dây (thông dụng) | Kết nối           |
|----------------------|-------------------|
| Đỏ                   | 5V hoặc 3.3V      |
| Đen                  | GND               |
| Vàng/Trắng           | Kênh A (GPIO)     |
| Xanh lá/Xám          | Kênh B (GPIO)     |
| (5 dây: +, -, M+, M-)| Dây motor         |

> ℹ️ JGB37-520 PPR = 333 xung/vòng (tốc độ thực sau hộp số).
> Với chế độ full quadrature (A+B), độ phân giải = 333 × 4 = 1332 xung/vòng.

---

## Bố cục bánh Mecanum (QUAN TRỌNG!)

```
    FL (/)    FR (\)
    ┌─────────────┐
    │  X  ROBOT   │  → Hướng tiến
    └─────────────┘
    RL (\)    RR (/)
```

- Bánh **FL** và **RR**: Con lăn nghiêng `/` (45°)
- Bánh **FR** và **RL**: Con lăn nghiêng `\` (-45°)

> ⚠️ Lắp sai hướng nghiêng sẽ khiến xe không thể trượt ngang!

---

## Nguồn điện

```
LiPo 3S (11.1V) ──┬── BTS7960 (12V in) ──→ Motor 12V
                   └── Buck Converter 5V ──→ Pi 5 + ESP32-S3
```

- Dùng **tụ lọc 1000µF** giữa nguồn và BTS7960 để giảm nhiễu.
- **KHÔNG** cấp nguồn 12V trực tiếp cho ESP32 hoặc Pi5.
- Dùng **dây AWG 14-16** cho mạch nguồn motor (dòng cao).

---

## Cài đặt phần mềm

### ESP32-S3 (PlatformIO)
```bash
# Cài PlatformIO CLI
pip install platformio

# Build và upload
cd mecanum_robot/esp32_firmware
pio run --target upload
pio device monitor    # Xem Serial Monitor
```

### Raspberry Pi 5
```bash
cd mecanum_robot/pi5_controller

# Điều khiển bàn phím
python3 teleop_keyboard.py

# Chạy demo tự động
python3 auto_navigation.py
```

### Bước thiết lập quan trọng
1. **Sửa WiFi**: Mở `esp32_firmware/src/config.h`, điền `WIFI_SSID` và `WIFI_PASSWORD`
2. **Sửa IP ESP32**: Mở `pi5_controller/robot_driver.py`, điền `ESP32_IP` đúng IP của kit
3. Upload firmware lên ESP32-S3
4. Kiểm tra Serial Monitor để lấy IP của ESP32
5. Chạy script Python trên Pi5

---

## Tần số hoạt động

| Module         | Tần số  |
|----------------|---------|
| PID Control    | 50 Hz   |
| Odometry Pub   | 20 Hz   |
| UDP Cmd Send   | 50 Hz   |
| Motor PWM      | 20 kHz  |

---

## Debug & Tinh chỉnh PID

Nếu motor dao động hoặc không đạt tốc độ mục tiêu:
1. Giảm `PID_KP` (chia đôi) cho đến khi hết dao động
2. Tăng dần `PID_KI` để giảm sai số tĩnh
3. `PID_KD` thường rất nhỏ, tăng nếu cần giảm overshoot

Thông số trong `config.h`:
```cpp
#define PID_KP  2.5   // Tỷ lệ
#define PID_KI  0.8   // Tích phân
#define PID_KD  0.05  // Vi phân
```
