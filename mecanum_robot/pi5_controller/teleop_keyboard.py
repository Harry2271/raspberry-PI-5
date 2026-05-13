#!/usr/bin/env python3
"""
teleop_keyboard.py
==================
Điều khiển xe Mecanum qua bàn phím từ Raspberry Pi 5.

Phím điều khiển:
    W / S       : Tiến / Lùi
    A / D       : Trượt ngang Trái / Phải
    Q / E       : Quay tại chỗ Trái / Phải
    Z           : Chéo Trái-Trước   C: Chéo Phải-Trước
    X / V       : Chéo Trái-Sau / Chéo Phải-Sau
    SPACE       : Dừng khẩn cấp
    +/-         : Tăng/Giảm tốc độ
    P           : In trạng thái robot
    ESC/CTRL+C  : Thoát
"""

import sys
import time
import math

# Xử lý tty trên Linux/Pi5
import tty
import termios

from robot_driver import MecanumRobot

# Cấu hình tốc độ mặc định
DEFAULT_VX = 0.3    # m/s
DEFAULT_VY = 0.3    # m/s
DEFAULT_VZ = 0.8    # rad/s
SPEED_STEP = 0.05   # Bước tăng/giảm tốc độ


def get_key(fd):
    """Đọc 1 ký tự từ stdin không cần Enter."""
    ch = sys.stdin.read(1)
    return ch


def print_status(robot: MecanumRobot, vx, vy, vz):
    state = robot.state
    x, y, th = robot.get_pose()
    print(f"\r[Cmd] Vx={vx:+.2f} Vy={vy:+.2f} Vz={vz:+.2f}  "
          f"[Pose] X={x:.3f}m Y={y:.3f}m θ={th:.1f}°  "
          f"[RPM] FL={state.fl_rpm:.0f} FR={state.fr_rpm:.0f} "
          f"RL={state.rl_rpm:.0f} RR={state.rr_rpm:.0f}  "
          f"[{'OK' if robot.is_connected else 'LOST'}]", end="")


def main():
    print("=" * 60)
    print("  Mecanum Robot Teleop - Keyboard Controller")
    print("=" * 60)
    print("  W/S=Tiến/Lùi | A/D=Ngang | Q/E=Quay | SPACE=Dừng")
    print("  Z/C=Chéo trước | X/V=Chéo sau | +/-=Tốc độ | ESC=Thoát")
    print("=" * 60)

    speed_scale = 1.0
    vx_max = DEFAULT_VX
    vy_max = DEFAULT_VY
    vz_max = DEFAULT_VZ

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    with MecanumRobot() as robot:
        time.sleep(0.5)
        print(f"[INFO] Kết nối: {'OK' if robot.is_connected else 'Chờ...'}")

        try:
            tty.setraw(fd)
            while True:
                key = get_key(fd)

                if key in ('\x1b', '\x03'):   # ESC hoặc Ctrl+C
                    break
                elif key == ' ':
                    robot.stop()
                elif key == 'w':
                    robot.move(vx=vx_max * speed_scale)
                elif key == 's':
                    robot.move(vx=-vx_max * speed_scale)
                elif key == 'a':
                    robot.move(vy=-vy_max * speed_scale)
                elif key == 'd':
                    robot.move(vy=vy_max * speed_scale)
                elif key == 'q':
                    robot.rotate_ccw(vz_max * speed_scale)
                elif key == 'e':
                    robot.rotate_cw(vz_max * speed_scale)
                elif key == 'z':
                    robot.move(vx=vx_max * speed_scale, vy=-vy_max * speed_scale)
                elif key == 'c':
                    robot.move(vx=vx_max * speed_scale, vy=vy_max * speed_scale)
                elif key == 'x':
                    robot.move(vx=-vx_max * speed_scale, vy=-vy_max * speed_scale)
                elif key == 'v':
                    robot.move(vx=-vx_max * speed_scale, vy=vy_max * speed_scale)
                elif key == '+' or key == '=':
                    speed_scale = min(2.0, speed_scale + SPEED_STEP / DEFAULT_VX)
                elif key == '-' or key == '_':
                    speed_scale = max(0.1, speed_scale - SPEED_STEP / DEFAULT_VX)
                elif key == 'p':
                    print()  # Xuống hàng
                    state = robot.state
                    print(f"[State] RPM: FL={state.fl_rpm:.1f} FR={state.fr_rpm:.1f} "
                          f"RL={state.rl_rpm:.1f} RR={state.rr_rpm:.1f}")
                    x, y, th = robot.get_pose()
                    print(f"[Pose]  X={x:.4f}m Y={y:.4f}m θ={th:.2f}°")
                elif key == 'r':
                    # Reset odometry (gửi lệnh reset)
                    print("\n[INFO] Odometry Reset (restart ESP32 để reset hoàn toàn)")

                # Hiển thị trạng thái
                cmd = robot._cmd
                print_status(robot, cmd.vx, cmd.vy, cmd.vz)

        except Exception as e:
            print(f"\n[ERROR] {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            robot.stop()
            print("\n[INFO] Đã dừng robot. Thoát.")


if __name__ == "__main__":
    main()
