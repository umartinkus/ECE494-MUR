import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import struct

def float_to_bytes(value, endian='<'):
    format_string = f'{endian}f'
    packed_bytes = struct.pack(format_string, value)
    return packed_bytes

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        bts = []  # bytes to send
        for ax in msg.axes:
            ax_bytes = float_to_bytes(ax)
            bts.append([hex(b) for b in ax_bytes])

        self.get_logger().info(f'{bts}')


def main(args=None):
    rclpy.init(args=args)

    node = JoySubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
