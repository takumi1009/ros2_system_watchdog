# SPDX-FileCopyrightText: 2025 Komiya Takumi
# SPDX-License-Identifier: Apache-2.0

"""Node for monitoring system CPU usage."""

import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SystemWatchdog(Node):
    """Main node class for monitoring CPU usage."""

    def __init__(self):
        """Initialize the node and create a timer."""
        super().__init__('system_watchdog')
        self.timer = self.create_timer(1.0, self.check_cpu_usage)
        self.publisher_ = self.create_publisher(Float32, 'cpu_usage', 10)
        self.get_logger().info('System Watchdog Node has started.')

    def check_cpu_usage(self):
        """Check current CPU usage and publish it to a topic."""
        cpu_percent = psutil.cpu_percent(interval=None)
        msg = Float32()
        msg.data = float(cpu_percent)
        self.publisher_.publish(msg)

        if cpu_percent > 80.0:
            self.get_logger().warn(f'High CPU Usage Detected: {cpu_percent}%')
        else:
            self.get_logger().info(f'CPU Usage: {cpu_percent}%')


def main(args=None):
    """Run the main loop for the watchdog node."""
    rclpy.init(args=args)
    node = SystemWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
