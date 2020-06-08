import math
import time
from typing import Any

import rclpy
import rclpy.node
import rclpy.publisher
import std_srvs.srv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class FollowSquare(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(FollowSquare, self).__init__("do_square")
        self.create_client(std_srvs.srv.Empty, 'is_ready').wait_for_service()
        speed = self.declare_parameter('speed', 0.05).value
        axis_length = self.declare_parameter('axis_length', 0.0935).value
        side_length = self.declare_parameter('side', 0.6).value
        self.get_logger().info('Init open loop square')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_subscription(Odometry, 'odom', self.update_odom, 1)
        self.pose = (0.0, 0.0, 0.0)
        self.sleep(1)
        rclpy.spin_once(self)
        self.get_logger().info(
            f'Start square with speed {speed:.3f} m/s and side {side_length:.3f} m')
        for _ in range(4):
            self.get_logger().info(f'Side from {self.pose}')
            self.advance(side_length, speed=speed)
            self.get_logger().info(f'Turn from {self.pose}')
            self.turn(math.pi / 2, angular_speed=(speed / axis_length))
            self.sleep(1)
        self.get_logger().info(f'Done at {self.pose}')
        self.sleep(1)

    def sleep(self, dt: float) -> None:
        time.sleep(dt)

    def update_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        angle = 2.0 * math.asin(msg.pose.pose.orientation.z)
        self.pose = (position.x, position.y, angle)

    def stop(self) -> None:
        self.pub.publish(Twist())
        rclpy.spin_once(self)

    def advance(self, length: float, speed: float) -> None:
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)
        self.sleep(length / speed)
        self.stop()

    def turn(self, angle: float, angular_speed: float) -> None:
        msg = Twist()
        msg.angular.z = angular_speed
        self.pub.publish(msg)
        self.sleep(angle / angular_speed)
        self.stop()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    FollowSquare()
