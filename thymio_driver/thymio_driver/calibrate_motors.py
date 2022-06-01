import csv
import enum
import os
from dataclasses import dataclass
from datetime import datetime as dt
from typing import Any, Dict, List, NamedTuple, Optional
try:
    from typing import Literal
except ImportError:
    from typing_extensions import Literal  # type: ignore


import numpy as np
import yaml
from scipy.optimize import curve_fit

import rclpy.node
from asebaros_msgs.msg import Event as AsebaEvent
from rclpy.time import Time
from std_msgs.msg import Bool
from thymio_msgs.msg import Led, SystemSound


class LineState(enum.Enum):
    IN = 0
    OFF = 1
    UNKNOWN = 2


class CalibrationResult(NamedTuple):
    params: List[float]
    kind: str

    def as_dict(self) -> Dict[str, Any]:
        return {'kind': self.kind, 'q': self.params}


def calibrate(samples: np.ndarray) -> CalibrationResult:
    def f(x: float, a: float, b: float) -> float:
        return a * x + b * x ** 2
    motor_speed = samples[:, 0]
    speed = samples[:, 1]
    popt, pcov = curve_fit(f, motor_speed, speed)
    return CalibrationResult(params=popt.tolist(), kind='quadratic')


@dataclass
class Line:
    state: LineState = LineState.UNKNOWN
    in_at_time: Optional[float] = None


class CalibrationState(enum.Enum):
    waiting_button = 0
    waiting_ground = 1
    running = 2


DEFAULT_MOTOR_SPEEDS: List[float] = [20, 40, 60, 80, 100, 120, 140, 160, 200, 250, 300, 350,
                                     400, 450]


class Calibration(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super(Calibration, self).__init__('motor_calibration')
        self.clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.ROS_TIME)
        self.gap = self.declare_parameter('gap', 100).value
        self.axis_length: float = self.declare_parameter('axis_length', 0.0935).value
        self.target_number_of_samples = self.declare_parameter('number_of_samples', 1).value
        self.motor_speeds = self.declare_parameter('motor_speeds', DEFAULT_MOTOR_SPEEDS).value
        self.get_logger().warn(f'{self.motor_speeds}, {type(self.motor_speeds)}')
        self.save_to = self.declare_parameter('save_to', '').value
        os.makedirs(self.save_to, exist_ok=True)
        self.should_save_samples = self.declare_parameter('save_samples', False).value
        self.sample_folder = os.path.join(self.save_to, dt.now().isoformat())
        self.state = CalibrationState.waiting_button
        self.pick: Optional[float] = None
        self._motor_speed = 0
        self.sound_pub = self.create_publisher(SystemSound, 'sound/play/system', 1)
        self.pub = self.create_publisher(AsebaEvent, 'aseba/events/set_speed', 1)
        self.led_pub = self.create_publisher(Led, 'led', 1)
        self.create_subscription(AsebaEvent, 'aseba/events/ground', self.update_state, 1)
        self.create_subscription(Bool, 'buttons/center', self.button, 1)
        for motor in self.declare_parameter('motors', ['right', 'left']).value:
            self.calibrate(motor=motor)
        self.get_logger().info('Motor calibration was successful')

    def button(self, msg: Bool) -> None:
        self.get_logger().info(f'button {self.state}')
        if self.state == CalibrationState.waiting_button:
            self.state = CalibrationState.waiting_ground
            self.get_logger().info(f'switch to {self.state}')

    def ping(self, sound: SystemSound = SystemSound.TARGET_OK) -> None:
        self.sound_pub.publish(SystemSound(sound=sound))

    def sleep(self, dt: float) -> None:
        # self.get_logger().info(f'will sleep for {dt}')
        rclpy.spin_once(self, timeout_sec=dt)
        # self.get_logger().info(f'has slept for {dt}')

    def calibrate(self, motor: Literal['left', 'right'] = 'right', angle: float = (2 * np.pi)
                  ) -> None:
        self.line = Line()
        self.pick = None
        self._samples = [[0.0, 0.0]]
        self.motor = motor
        self.motor_speed = 0
        self.angle = angle
        standing_wheel = 'right' if self.motor == 'left' else 'left'
        self.get_logger().info(
            (f'Place the robot with the {standing_wheel} wheel at the center of the T '
             'and press the central button when the robot is ready'))
        self.ping()
        self.state = CalibrationState.waiting_button
        while self.state != CalibrationState.waiting_ground:
            self.sleep(1)
        self.sleep(3)
        while self.state == CalibrationState.waiting_ground:
            self.sleep(1)
        self.get_logger().info(f'Start calibrating {self.motor} motor using thresholds {self.ths}')
        _n = 1
        for motor_speed in self.motor_speeds:
            self.motor_speed = motor_speed
            self.get_logger().info(f'Set motor speed to {motor_speed}')
            _n += self.target_number_of_samples
            while len(self._samples) < _n:
                self.sleep(0.01)
        self.motor_speed = 0
        self.get_logger().debug(
            f'Calculate with samples {self._samples}')
        if self.should_save_samples:
            self.save_samples()
        samples = np.array(self._samples)
        results = calibrate(samples)
        self.get_logger().info(f'Calibration of motor {self.motor} done: {results}')
        self.save_calibration(results)

    def save_samples(self) -> None:
        os.makedirs(self.sample_folder, exist_ok=True)
        path = os.path.join(self.sample_folder, f'{self.motor}.csv')
        with open(path, 'w') as f:
            writer = csv.writer(f, delimiter=',')
            writer.writerow(['motor_speed', 'speed'])
            writer.writerows(self._samples)

    def save_calibration(self, result: CalibrationResult) -> None:
        name = f'{self.motor}.yaml'
        path = os.path.join(self.sample_folder, name)
        with open(path, 'w') as f:
            yaml.dump(result.as_dict(), f)
        # l_path = os.path.join(os.path.basename(self.sample_folder), name)
        # t_path = os.path.join(self.sample_folder, '..', name)
        # self.get_logger().info(f'Create symlink {t_path} -> {path}')
        # try:
        #     os.symlink(l_path, t_path)
        # except OSError:
        #     os.remove(t_path)
        #     os.symlink(l_path, t_path)

    @property
    def motor_speed(self) -> int:
        return self._motor_speed

    @motor_speed.setter
    def motor_speed(self, value: int) -> None:
        if self._motor_speed == value:
            return
        self._motor_speed = value
        if self.motor == 'right':
            data = [0, value]
        else:
            data = [value, 0]
        self.pub.publish(AsebaEvent(data=data))
        c = 8 * value / 500
        leds = [max(0, min(1, c - i)) for i in range(8)]
        self.led_pub.publish(Led(id=Led.CIRCLE, values=leds))

    def speed(self, period: float) -> float:
        return self.angle / period * self.axis_length

    def add_pick(self, pick: float) -> None:
        if self.pick is not None:
            dt = pick - self.pick
            self._samples.append([self.motor_speed, self.speed(dt)])
            self.get_logger().info(f'Added sample {dt:.3f} -> {self._samples[-1]}')
        self.pick = pick

    def update_state(self, msg: AsebaEvent) -> None:

        if self.state == CalibrationState.waiting_button:
            return

        if self.state == CalibrationState.waiting_ground:
            self.ths = [d - self.gap for d in msg.data]
            self.state = CalibrationState.running
            return

        if self.motor_speed == 0:
            return
        line = self.line
        if self.motor == 'right':
            v = msg.data[0]
            th = self.ths[0]
        else:
            v = msg.data[1]
            th = self.ths[1]
        self.get_logger().info(f'update_state {line.state} {v} {th} -> ?')
        stamp = Time.from_msg(msg.stamp).nanoseconds * 1e-9
        if v > th and line.state == LineState.UNKNOWN:
            line.state = LineState.OFF
        if v < th and line.state == LineState.OFF:
            line.state = LineState.IN
            line.in_at_time = stamp
        if v > th and line.state == LineState.IN:
            line.state = LineState.OFF
            pick = (stamp + line.in_at_time) / 2
            self.get_logger().info(f"Line after {pick} s")
            self.add_pick(pick)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    Calibration()
