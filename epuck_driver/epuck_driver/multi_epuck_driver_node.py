from typing import Any

import rclpy
import rclpy.executors
from thymio_driver.manager import Manager

from .epuck_driver_node import EpuckDriver


class EpuckManager(Manager):  # type: ignore

    _drivers = {'epuck0': EpuckDriver}


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    manager = EpuckManager(executor)
    executor.spin()
    manager.destroy_node()
    rclpy.shutdown()
