import math
from typing import Any, Callable, Optional, List

import rclpy
import rclpy.time
from thymio_msgs.msg import Led
from asebaros_msgs.msg import AsebaEvent
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from .base import BaseDriver


def deg2rad(v: float) -> float:
    return v * math.pi / 180

# range_max: 0.12


class EpuckDriver(BaseDriver):
    kind = 'e-puck0'
    axis_length = 0.051
    wheel_radius = 0.021
    max_aseba_speed = 1000
    motor_calibration = {'kind': 'quadratic', 'parameters': [0.000128, 0.0], 'deadband': 0}
    proximity_names = [f'ring_{i}' for i in range(8)]
    proximity_calibration = {'parameters': [3731.0, 0.003, 0.00007], 'kind': 'power',
                             'range_max': 0.08, 'range_min': 0.003, 'fov': 0.3}
    laser_angles = {f'ring_{i}': deg2rad(theta) for i, theta
                    in enumerate([-18, -45, -90, -142, 142, 90, 45, 18])}
    laser_shift = 0.033

    def init(self) -> None:
        self.aseba_led_publisher = self.create_publisher(AsebaEvent, self._aseba('set_led'), 6)
        self.image_publisher = self.create_publisher(Image, self._ros('image_raw'), 1)
        self.image = Image()
        self.image.header.frame_id = self._ros('camera_link')
        self.image.height = 1
        self.image.width = 60
        self.image.encoding = 'rgb8'
        self.image.step = 3 * 60
        self.image_channels: List[List[float]] = []
        self.image_first_channel_stamp: Optional[float] = None
        self.create_subscription(AsebaEvent, self._aseba('image'), self.on_image, 3)
        self.create_subscription(Led, self._ros('led'), self.on_led, 6)
        self.create_subscription(Led, self._ros('led/body'), self.on_single_led(1), 6)
        self.create_subscription(Led, self._ros('led/torch'), self.on_single_led(2), 6)

    def on_led(self, msg: Led) -> None:
        # if Led.CIRCLE != msg.id:
        #     self.get_logger('warn')('The e-puck only support CIRCLE leds.')
        #     return
        num = 8
        if num <= len(msg.values):
            data = [0] + [1 if v > 0 else 0 for v in msg.values[:8]]
            data += [0] * (9 - len(data))
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=data))

    def on_single_led(self, index: int) -> Callable[[Bool], None]:
        def cb(msg: Bool) -> None:
            data = [index, 1 if msg.data else 0]
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=data))
        return cb

    def on_image(self, msg: AsebaEvent) -> None:
        stamp = rclpy.time.Time.from_msg(msg.stamp).nanoseconds * 1e-6
        if self.image_first_channel_stamp is None:
            self.image_first_channel_stamp = stamp
            self.image_channels = [msg.data]
        else:
            dt = stamp - self.image_first_channel_stamp
            self.get_logger().debug(f'new channel after {dt:.0f} ms')
            if dt < 50:
                self.image_channels.append(msg.data)
            else:
                self.image_first_channel_stamp = stamp
                self.image_channels = [msg.data]
        if len(self.image_channels) == 3:
            self.image.data = list(sum(zip(*self.image_channels), ()))
            self.image_publisher.publish(self.image)
            self.image_first_channel_stamp = None


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    driver = EpuckDriver(namespace='', standalone=True)
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()
