#!/usr/bin/env python3
"""
robot_driver.py
===============
Class điều khiển xe Mecanum qua UDP từ Raspberry Pi 5.

Giao thức:
  - Gửi (Pi5 → ESP32): JSON {"vx": 0.3, "vy": 0.0, "vz": 0.1, "seq": N}
  - Nhận (ESP32 → Pi5): JSON {"fl":..., "fr":..., "rl":..., "rr":...,
                               "x":..., "y":..., "th":..., "seq":N}

Cài đặt:
  pip install -r requirements.txt
"""

import socket
import json
import threading
import time
import logging
from dataclasses import dataclass, field
from typing import Optional

# ── Cấu hình ────────────────────────────────────────────────
ESP32_IP          = "192.168.1.100"   # ← Thay bằng IP của ESP32
ESP32_UDP_PORT    = 8888              # Port ESP32 lắng nghe
PI5_FEEDBACK_PORT = 8889              # Port Pi5 nhận feedback
CMD_INTERVAL      = 0.02             # Gửi lệnh mỗi 20ms (50Hz)
FEEDBACK_TIMEOUT  = 1.0              # Timeout feedback (s)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("MecanumRobot")


# ── Data Structures ─────────────────────────────────────────
@dataclass
class RobotState:
    """Trạng thái hiện tại của robot (cập nhật từ feedback ESP32)"""
    fl_rpm:  float = 0.0
    fr_rpm:  float = 0.0
    rl_rpm:  float = 0.0
    rr_rpm:  float = 0.0
    x:       float = 0.0  # m
    y:       float = 0.0  # m
    theta:   float = 0.0  # rad
    seq:     int   = 0
    last_update: float = 0.0
    connected: bool = False


@dataclass
class VelocityCommand:
    vx: float = 0.0   # m/s tiến/lùi
    vy: float = 0.0   # m/s sang ngang
    vz: float = 0.0   # rad/s quay


# ── Lớp điều khiển chính ────────────────────────────────────
class MecanumRobot:
    """
    Giao diện điều khiển xe tự hành Mecanum.
    
    Sử dụng:
        robot = MecanumRobot()
        robot.connect()
        robot.move(vx=0.2, vy=0.0, vz=0.0)   # Tiến 0.2 m/s
        time.sleep(2)
        robot.stop()
        robot.disconnect()
    """

    def __init__(self,
                 esp32_ip:   str = ESP32_IP,
                 esp32_port: int = ESP32_UDP_PORT,
                 listen_port: int = PI5_FEEDBACK_PORT):
        self._esp32_addr   = (esp32_ip, esp32_port)
        self._listen_port  = listen_port
        self._cmd          = VelocityCommand()
        self._state        = RobotState()
        self._seq          = 0
        self._lock         = threading.Lock()
        self._running      = False

        # Sockets
        self._tx_sock: Optional[socket.socket] = None
        self._rx_sock: Optional[socket.socket] = None

        # Threads
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None

    # ── Public API ───────────────────────────────────────────

    def connect(self) -> bool:
        """Khởi tạo kết nối UDP và bắt đầu các thread."""
        try:
            # TX socket (gửi lệnh)
            self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)

            # RX socket (nhận feedback)
            self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._rx_sock.bind(("", self._listen_port))
            self._rx_sock.settimeout(0.5)

            self._running = True

            # Bắt đầu threads
            self._tx_thread = threading.Thread(
                target=self._tx_loop, daemon=True, name="RobotTX")
            self._rx_thread = threading.Thread(
                target=self._rx_loop, daemon=True, name="RobotRX")
            self._tx_thread.start()
            self._rx_thread.start()

            log.info(f"Kết nối đến ESP32 tại {self._esp32_addr[0]}:{self._esp32_addr[1]}")
            log.info(f"Lắng nghe feedback tại port {self._listen_port}")
            return True

        except Exception as e:
            log.error(f"Lỗi kết nối: {e}")
            return False

    def disconnect(self):
        """Dừng robot và ngắt kết nối."""
        self.stop()
        time.sleep(0.1)
        self._running = False
        if self._tx_sock:
            self._tx_sock.close()
        if self._rx_sock:
            self._rx_sock.close()
        log.info("Đã ngắt kết nối.")

    def move(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0):
        """
        Điều khiển chuyển động.
        
        Args:
            vx:  Tiến (+) / Lùi (-)  đơn vị m/s   (max ≈ 1.67 m/s)
            vy:  Phải (+) / Trái (-) đơn vị m/s
            vz:  Quay CW (+) / CCW (-) đơn vị rad/s
        """
        with self._lock:
            self._cmd.vx = float(vx)
            self._cmd.vy = float(vy)
            self._cmd.vz = float(vz)

    def stop(self):
        """Dừng toàn bộ chuyển động."""
        self.move(0.0, 0.0, 0.0)

    def forward(self, speed: float = 0.3):
        """Tiến thẳng."""
        self.move(vx=speed)

    def backward(self, speed: float = 0.3):
        """Lùi thẳng."""
        self.move(vx=-speed)

    def strafe_right(self, speed: float = 0.3):
        """Trượt ngang sang phải."""
        self.move(vy=speed)

    def strafe_left(self, speed: float = 0.3):
        """Trượt ngang sang trái."""
        self.move(vy=-speed)

    def rotate_cw(self, speed: float = 0.5):
        """Quay tại chỗ thuận chiều kim đồng hồ."""
        self.move(vz=speed)

    def rotate_ccw(self, speed: float = 0.5):
        """Quay tại chỗ ngược chiều kim đồng hồ."""
        self.move(vz=-speed)

    def diagonal(self, direction: str = "fr", speed: float = 0.3):
        """Di chuyển chéo. direction: 'fr','fl','br','bl'"""
        _map = {
            "fr": ( speed,  speed, 0.0),
            "fl": ( speed, -speed, 0.0),
            "br": (-speed,  speed, 0.0),
            "bl": (-speed, -speed, 0.0),
        }
        self.move(*_map.get(direction, (0.0, 0.0, 0.0)))

    @property
    def state(self) -> RobotState:
        """Trạng thái mới nhất của robot."""
        with self._lock:
            return self._state

    @property
    def is_connected(self) -> bool:
        """True nếu nhận feedback từ ESP32 trong 1 giây gần đây."""
        with self._lock:
            return (time.time() - self._state.last_update) < FEEDBACK_TIMEOUT

    def get_pose(self) -> tuple:
        """Trả về (x, y, theta_deg)."""
        import math
        s = self.state
        return (s.x, s.y, math.degrees(s.theta))

    # ── Private: Thread TX ───────────────────────────────────
    def _tx_loop(self):
        """Gửi lệnh vận tốc liên tục với tần số 50Hz."""
        while self._running:
            t0 = time.monotonic()

            with self._lock:
                packet = {
                    "vx":  round(self._cmd.vx, 4),
                    "vy":  round(self._cmd.vy, 4),
                    "vz":  round(self._cmd.vz, 4),
                    "seq": self._seq,
                }
                self._seq = (self._seq + 1) & 0xFFFF

            try:
                data = json.dumps(packet, separators=(",", ":")).encode()
                self._tx_sock.sendto(data, self._esp32_addr)
            except Exception as e:
                if self._running:
                    log.warning(f"TX lỗi: {e}")

            # Chờ đủ 20ms
            elapsed = time.monotonic() - t0
            sleep_t = CMD_INTERVAL - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── Private: Thread RX ───────────────────────────────────
    def _rx_loop(self):
        """Nhận và xử lý feedback từ ESP32."""
        while self._running:
            try:
                data, addr = self._rx_sock.recvfrom(512)
                pkt = json.loads(data.decode())

                with self._lock:
                    self._state.fl_rpm      = float(pkt.get("fl", 0))
                    self._state.fr_rpm      = float(pkt.get("fr", 0))
                    self._state.rl_rpm      = float(pkt.get("rl", 0))
                    self._state.rr_rpm      = float(pkt.get("rr", 0))
                    self._state.x           = float(pkt.get("x", 0))
                    self._state.y           = float(pkt.get("y", 0))
                    self._state.theta       = float(pkt.get("th", 0))
                    self._state.seq         = int(pkt.get("seq", 0))
                    self._state.last_update = time.time()
                    self._state.connected   = True

            except socket.timeout:
                pass
            except json.JSONDecodeError as e:
                log.warning(f"JSON lỗi: {e}")
            except Exception as e:
                if self._running:
                    log.warning(f"RX lỗi: {e}")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    def __repr__(self):
        s = self.state
        return (f"MecanumRobot(x={s.x:.3f}m, y={s.y:.3f}m, "
                f"θ={s.theta:.3f}rad, connected={self.is_connected})")
