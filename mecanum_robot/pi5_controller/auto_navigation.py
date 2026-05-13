#!/usr/bin/env python3
"""
auto_navigation.py
==================
Ví dụ điều hướng tự động đơn giản cho xe Mecanum.

Demo bao gồm:
  1. Di chuyển hình vuông
  2. Di chuyển hình chữ Thập (Cross)
  3. Chuyển động 360° tại chỗ
"""

import time
import math
from robot_driver import MecanumRobot


class NavigationDemo:
    def __init__(self, robot: MecanumRobot):
        self.robot = robot

    def _move_time(self, vx=0.0, vy=0.0, vz=0.0, duration=1.0, msg=""):
        """Di chuyển trong thời gian cố định."""
        if msg:
            print(f"  → {msg}")
        self.robot.move(vx=vx, vy=vy, vz=vz)
        time.sleep(duration)
        self.robot.stop()
        time.sleep(0.2)

    def square_demo(self, side_speed=0.2, side_time=2.0):
        """Di chuyển theo hình vuông."""
        print("\n[DEMO] Hình Vuông (4 cạnh)")
        moves = [
            ("Tiến", 0.2,  0.0, 0.0),
            ("Phải", 0.0,  0.2, 0.0),
            ("Lùi",  -0.2, 0.0, 0.0),
            ("Trái", 0.0, -0.2, 0.0),
        ]
        for label, vx, vy, vz in moves:
            self._move_time(vx=vx, vy=vy, vz=vz,
                            duration=side_time, msg=label)

    def cross_demo(self, speed=0.2, duration=1.5):
        """Di chuyển theo hình chữ thập."""
        print("\n[DEMO] Hình Chữ Thập")
        moves = [
            ("Tiến",  speed,  0.0, 0.0),
            ("Lùi",  -speed,  0.0, 0.0),
            ("Phải",  0.0,   speed, 0.0),
            ("Trái",  0.0,  -speed, 0.0),
        ]
        for label, vx, vy, vz in moves:
            self._move_time(vx=vx, vy=vy, vz=vz,
                            duration=duration, msg=label)

    def spin_demo(self, vz=0.8, duration=4.0):
        """Quay 360° tại chỗ."""
        print("\n[DEMO] Quay 360° tại chỗ")
        self._move_time(vz=vz, duration=duration, msg="Quay CW")

    def diagonal_demo(self, speed=0.2, duration=2.0):
        """Di chuyển chéo - đặc trưng của bánh Mecanum."""
        print("\n[DEMO] Di chuyển chéo (Mecanum đặc trưng)")
        moves = [
            ("Chéo Trước-Phải", speed,  speed, 0.0),
            ("Chéo Sau-Trái",  -speed, -speed, 0.0),
            ("Chéo Trước-Trái", speed, -speed, 0.0),
            ("Chéo Sau-Phải",  -speed,  speed, 0.0),
        ]
        for label, vx, vy, vz in moves:
            self._move_time(vx=vx, vy=vy, vz=vz,
                            duration=duration, msg=label)

    def run_all(self):
        """Chạy tất cả các demo."""
        print("=" * 50)
        print("  Mecanum Robot - Auto Navigation Demo")
        print("=" * 50)
        print(f"[INFO] Chờ kết nối...", end=" ")

        # Chờ kết nối tối đa 5 giây
        for _ in range(10):
            if self.robot.is_connected:
                break
            time.sleep(0.5)

        if not self.robot.is_connected:
            print("THẤT BẠI! Kiểm tra kết nối WiFi và IP ESP32.")
            return

        print("OK!")
        x, y, th = self.robot.get_pose()
        print(f"[Pose khởi đầu] X={x:.3f}m Y={y:.3f}m θ={th:.1f}°")

        try:
            self.square_demo()
            time.sleep(1.0)

            self.cross_demo()
            time.sleep(1.0)

            self.diagonal_demo()
            time.sleep(1.0)

            self.spin_demo()

        except KeyboardInterrupt:
            print("\n[INTERRUPT] Người dùng dừng!")
        finally:
            self.robot.stop()
            x, y, th = self.robot.get_pose()
            print(f"\n[Pose kết thúc] X={x:.3f}m Y={y:.3f}m θ={th:.1f}°")
            print("[DEMO] Hoàn tất!")


def main():
    with MecanumRobot() as robot:
        nav = NavigationDemo(robot)
        nav.run_all()


if __name__ == "__main__":
    main()
