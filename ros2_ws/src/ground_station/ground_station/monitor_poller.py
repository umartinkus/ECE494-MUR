import rclpy
from rclpy.node import Node

from custom_interfaces.msg import SPI

class MonitorPoller(Node):
    """ROS2 node that periodically polls for new data from the SPI monitor and updates the UI."""

    def __init__(self) -> None:
        """Create the ROS2 node and start polling loop."""
        super().__init__("monitor_poller")
        self.publisher_ = self.create_publisher(SPI, "/spi_monitor", 10)
        self.timer = self.create_timer(0.05, self._poll_spi_monitor)

    def _poll_spi_monitor(self) -> None:
        """Trigger an update of the SPI monitor UI."""
        self.get_logger().debug("Polling SPI monitor for new data...")
        msg = SPI()
        msg.synch = 0x55
        msg.syncl = 0x55
        msg.address = 99  # Address doesn't matter for polling - monitor will publish to /spi_receive
        msg.size = 0
        msg.crc = 0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    monitor_poller = MonitorPoller()
    rclpy.spin(monitor_poller)
    monitor_poller.destroy_node()
    rclpy.shutdown()