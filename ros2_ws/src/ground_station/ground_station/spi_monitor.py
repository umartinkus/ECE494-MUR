import struct
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import SPI


STATUS_ADDRESS = 0
SENSOR_ADDRESS = 1

STATUS_NAMES = (
    "i2c_bus_status",
    "spi_bus_status",
    "pwm_status",
    "imu1_status",
    "imu2_status",
    "ps_status",
)

ERROR_LABELS = {
    0: "UNINITIALIZED",
    1: "OK",
    2: "ERROR",
    3: "LEAK_DETECTED",
    4: "CRC_FAILED",
    5: "CONFIG_ERR",
    6: "TIMEOUT",
    99: "UNKNOWN",
}


class SpiMonitor(Node):
    def __init__(self) -> None:
        super().__init__("spi_monitor")

        self._root = tk.Tk()
        self._root.title("SPI Sensor Monitor")
        self._root.geometry("520x460")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._closed = False

        self._meta_var = tk.StringVar(value="Waiting for /spi_receive...")
        self._status_vars = {name: tk.StringVar(value="--") for name in STATUS_NAMES}
        self._sensor_vars = {
            "imu1_accel": tk.StringVar(value="--"),
            "imu1_gyro": tk.StringVar(value="--"),
            "imu1_mag": tk.StringVar(value="--"),
            "imu1_temp": tk.StringVar(value="--"),
            "imu2_accel": tk.StringVar(value="--"),
            "imu2_gyro": tk.StringVar(value="--"),
            "imu2_mag": tk.StringVar(value="--"),
            "imu2_temp": tk.StringVar(value="--"),
            "bar30_pressure": tk.StringVar(value="--"),
            "bar30_temp": tk.StringVar(value="--"),
        }

        self._build_ui()

        self.create_subscription(SPI, "/spi_receive", self._handle_spi, 10)
        self._root.after(20, self._pump)

    def _build_ui(self) -> None:
        frame = ttk.Frame(self._root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="SPI Receive Monitor", font=("TkDefaultFont", 14, "bold")).pack(anchor=tk.W)
        ttk.Label(frame, textvariable=self._meta_var).pack(anchor=tk.W, pady=(4, 10))

        status_box = ttk.LabelFrame(frame, text="System Status", padding=10)
        status_box.pack(fill=tk.X, pady=(0, 10))
        for row, name in enumerate(STATUS_NAMES):
            ttk.Label(status_box, text=name).grid(row=row, column=0, sticky=tk.W, padx=(0, 16), pady=2)
            ttk.Label(status_box, textvariable=self._status_vars[name]).grid(row=row, column=1, sticky=tk.W, pady=2)

        sensor_box = ttk.LabelFrame(frame, text="Sensor Data", padding=10)
        sensor_box.pack(fill=tk.BOTH, expand=True)
        labels = (
            ("IMU1 accel", "imu1_accel"),
            ("IMU1 gyro", "imu1_gyro"),
            ("IMU1 mag", "imu1_mag"),
            ("IMU1 temp", "imu1_temp"),
            ("IMU2 accel", "imu2_accel"),
            ("IMU2 gyro", "imu2_gyro"),
            ("IMU2 mag", "imu2_mag"),
            ("IMU2 temp", "imu2_temp"),
            ("BAR30 pressure", "bar30_pressure"),
            ("BAR30 temp", "bar30_temp"),
        )
        for row, (label, key) in enumerate(labels):
            ttk.Label(sensor_box, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 16), pady=2)
            ttk.Label(sensor_box, textvariable=self._sensor_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=2)

    def _handle_spi(self, msg: SPI) -> None:
        self._meta_var.set(
            f"Last packet: address={msg.address} size={msg.size} crc=0x{msg.crc:04X}"
        )

        payload = bytes(msg.data[:msg.size])

        if msg.address == STATUS_ADDRESS:
            self._update_status(payload)
        elif msg.address == SENSOR_ADDRESS:
            self._update_sensor(payload)
        else:
            self.get_logger().warning(
                f"Unhandled SPI address {msg.address} with payload size {msg.size}"
            )

    def _update_status(self, payload: bytes) -> None:
        if len(payload) != 24:
            self.get_logger().warning(f"Expected 24 status bytes, got {len(payload)}")
            return

        values = struct.unpack("<6I", payload)
        for name, value in zip(STATUS_NAMES, values):
            label = ERROR_LABELS.get(value, str(value))
            self._status_vars[name].set(f"{value} ({label})")

    def _update_sensor(self, payload: bytes) -> None:
        if len(payload) != 48:
            self.get_logger().warning(f"Expected 48 sensor bytes, got {len(payload)}")
            return

        values = struct.unpack("<20h2f", payload)

        imu1 = values[0:10]
        imu2 = values[10:20]
        bar30_pressure, bar30_temp = values[20:22]

        self._sensor_vars["imu1_accel"].set(self._fmt_vec3(imu1[0:3]))
        self._sensor_vars["imu1_gyro"].set(self._fmt_vec3(imu1[3:6]))
        self._sensor_vars["imu1_mag"].set(self._fmt_vec3(imu1[6:9]))
        self._sensor_vars["imu1_temp"].set(str(imu1[9]))

        self._sensor_vars["imu2_accel"].set(self._fmt_vec3(imu2[0:3]))
        self._sensor_vars["imu2_gyro"].set(self._fmt_vec3(imu2[3:6]))
        self._sensor_vars["imu2_mag"].set(self._fmt_vec3(imu2[6:9]))
        self._sensor_vars["imu2_temp"].set(str(imu2[9]))

        self._sensor_vars["bar30_pressure"].set(f"{bar30_pressure:.3f}")
        self._sensor_vars["bar30_temp"].set(f"{bar30_temp:.3f}")

    @staticmethod
    def _fmt_vec3(values: tuple[int, int, int] | list[int]) -> str:
        return f"x={values[0]} y={values[1]} z={values[2]}"

    def _pump(self) -> None:
        if self._closed:
            return

        rclpy.spin_once(self, timeout_sec=0.0)
        self._root.after(20, self._pump)

    def _on_close(self) -> None:
        self._closed = True
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpiMonitor()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
