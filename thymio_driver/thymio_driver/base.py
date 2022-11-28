import time
import yaml

from math import copysign, cos, sin, sqrt
from typing import Any, Dict, List, Optional, Tuple, TypeVar, TYPE_CHECKING, cast

try:
    from typing import TypedDict
except ImportError:
    from typing_extensions import TypedDict

import rclpy
import rclpy.node
import rclpy.parameter
import rclpy.time
import std_srvs.srv
# from asebaros_msgs.msg import AsebaEvent
from asebaros_msgs.msg import Event as AsebaEvent
# from asebaros_msgs.msg import Node as AsebaNode
from asebaros_msgs.srv import GetNodeList, LoadScript
from geometry_msgs.msg import (Quaternion, Transform, TransformStamped, Twist,
                               Vector3)
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState, LaserScan, Range
from std_msgs.msg import Header
from tf2_ros.transform_broadcaster import TransformBroadcaster

if TYPE_CHECKING:
    from .manager import Manager

T = TypeVar('T')


def sign(x: float) -> int:
    return int(copysign(1, x))


MotorCalibration = TypedDict('MotorCalibration',
                             {'parameters': List[float], 'kind': str, 'deadband': int})

ProximityCalibration = TypedDict('ProximityCalibration',
                                 {'parameters': List[float], 'kind': str,
                                  'range_max': float, 'range_min': float, 'fov': float})
Calibration = TypedDict(
    "Calibration",
    {'motor': Dict[str, MotorCalibration], 'proximity': Dict[str, ProximityCalibration]})


class MotorSpeedConversion:

    def __init__(self, calibration: MotorCalibration) -> None:
        if calibration['kind'] != 'quadratic':
            raise ValueError(f"Unknown calibration {calibration['kind']}")
        self.parameters = calibration['parameters']
        self.deadband = calibration['deadband']

    @property
    def parameters(self) -> List[float]:
        return [self.q0, self.q1]

    @parameters.setter
    def parameters(self, value: List[float]) -> None:
        self.q0, self.q1 = value[:2]

    def aseba_to_si(self, value: int) -> float:
        # if abs(value) < self.deadband:
        #     return 0.0
        return self.q0 * value + self.q1 * value * abs(value)

    def si_to_aseba(self, value: float) -> int:
        if self.q1 == 0:
            return int(round(value / self.q0))
        return int(round(0.5 * sign(value) *
                   (- self.q0 + sqrt(self.q0 ** 2 + 4 * self.q1 * abs(value))) / self.q1))


class ProximityConversion:

    def __init__(self, calibration: ProximityCalibration) -> None:
        if calibration['kind'] != 'power':
            raise ValueError(f"Unknown calibration {calibration['kind']}")
        self.parameters = calibration['parameters']
        self.range_max = calibration['range_max']
        self.range_min = calibration['range_min']
        self.fov = calibration['fov']

    @property
    def parameters(self) -> List[float]:
        return [self.max_value, self.x0, self.c]

    @parameters.setter
    def parameters(self, value: List[float]) -> None:
        self.max_value, self.x0, self.c = value[:3]

    def aseba_to_si(self, value: int) -> float:
        if value == 0:
            # HACK(Jerome) Galactic does not like +/- inf
            # return float('inf')
            # return self.range_max
            return -1.0
        if value >= self.max_value:
            # HACK(Jerome) Galactic does not like +/- inf
            return self.range_min
            # return -float('inf')
        dist = self.x0 + sqrt((self.x0 ** 2 - self.c) * (1 - self.max_value / value))
        return max(self.range_min, min(dist, self.range_max))


ProximitySensor = Tuple[rclpy.publisher.Publisher, Range, ProximityConversion]

_m = 2 ** 15


def delta(v1: int, v0: int) -> int:
    diff = v1 - v0
    if diff > _m:
        diff -= 2 * _m
    elif diff < -_m:
        diff += 2 * _m
    return diff


class BaseDriver(rclpy.node.Node):  # type: ignore
    kind: str
    axis_length: float
    wheel_radius: float
    max_aseba_speed: int
    motor_calibration: MotorCalibration
    proximity_names: List[str]
    proximity_calibration: ProximityCalibration
    laser_angles: Dict[str, float]
    laser_shift: float
    uid: int
    param_owner: rclpy.node.Node
    calibration: Calibration

    def __init__(self, namespace: str = '', manager: Optional['Manager'] = None, uid: int = -1) -> None:
        super().__init__("thymio_driver", namespace=namespace)
        if manager:
            self.param_owner = manager
        else:
            self.param_owner = self
        self.aseba_node = None
        self.uid = uid
        if not manager:
            self.uid, running = self.wait_for_aseba_node()
            if not running:
                self.load_script()
        self.clock = self.get_clock()
        if namespace:
            self.tf_prefix = namespace
        else:
            self.tf_prefix = self.declare_parameter('tf_prefix', '').value

        self.init_calibration()
        self.init_odometry()
        self.init_wheels()
        self.init_proximity()
        self.add_on_set_parameters_callback(self.param_callback)
        self.init()
        # REVIEW: not implemented yet in ROS2
        # self.on_shutdown(self.shutdown)
        self.create_service(std_srvs.srv.Empty, self._ros('is_ready'), self.ready)
        self.get_logger().info(f"{self.kind} is ready at namespace {self.get_namespace()}")

    def init_calibration(self) -> None:
        calibration_file = self.get_or_declare_parameter('calibration_file', '').value
        simulated = self.get_or_declare_parameter('simulation', False).value
        self.calibration = self.default_calibration(simulated)
        if calibration_file:
            self.get_logger().info(f"calibration_file {calibration_file}")
            try:
                with open(calibration_file, 'r') as f:
                    calibration = yaml.safe_load(f).get(self.uid, {})
                    self.get_logger().info(f"calibration from file {calibration}")
                    self.calibration.update(calibration)
            except FileNotFoundError:
                pass
        for group, v in self.calibration.items():
            for name, cal in cast(Dict[str, Dict], v).items():
                for param, value in cal.items():
                    self.declare_parameter(f'{group}.{name}.{param}', value)
        # self.get_logger().info(f"Loaded calibration {self.calibration}")

    def default_calibration(self, simulated: bool = False) -> Calibration:
        return {'motor': {}, 'proximity': {}}

    def get_or_declare_parameter(self, name: str, value: Any) -> rclpy.parameter.Parameter:
        if self.param_owner.has_parameter(name):
            return self.param_owner.get_parameter(name)
        return self.param_owner.declare_parameter(name, value)

    def param_callback(self, parameters: List[rclpy.parameter.Parameter]
                       ) -> SetParametersResult:
        # TODO(Jerome): move motor deadband
        for param in parameters:
            self.get_logger().debug(f'will set param {param.name} to {param.value}')
            for motor in ('left', 'right'):
                if f'motor_calibration.{motor}' in param.name:
                    # TODO: enforce bounds
                    conversion = self.motor_speed_conversion[motor]
                    if 'parameters' in param.name:
                        conversion.parameters = param.value
                    if 'deadband' in param.name:
                        conversion.deadband = param.value
                        self.get_logger().debug(
                            f"Changed {motor} motor deadband to {param.value}")
                        self.update_odom_convariance()
        response = SetParametersResult(successful=True)
        return response

    def update_odom_convariance(self) -> None:
        si_band = sum(c.aseba_to_si(c.deadband) for c in self.motor_speed_conversion.values()) / 2
        self.odom_msg.twist.covariance[0] = speed_cov = 0.5 * si_band ** 2
        self.odom_msg.twist.covariance[-1] = speed_cov / (self.axis_length ** 2)

    def load_script(self) -> None:
        # default_script = os.path.join(get_package_share_directory(
        #     'thymio_driver'), 'aseba', 'thymio_ros.aesl')
        default_script = ''
        script_path = self.get_or_declare_parameter('script', default_script).value
        if not script_path:
            self.get_logger().warn('Script not provided!')
            return

        self.get_logger().info(f'Try to load script at {script_path}')
        load_script_client = self.create_client(LoadScript, 'aseba/load_script')
        while not load_script_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('load_script service not available, waiting again...')
        req = LoadScript.Request(file_name=script_path)
        resp = load_script_client.call_async(req)
        rclpy.spin_until_future_complete(self, resp)

    def wait_for_aseba_node(self) -> Tuple[int, bool]:
        self.get_logger().info(
            f"Waiting for an Aseba node with name {self.kind}")
        get_aseba_nodes = self.create_client(GetNodeList, 'aseba/get_nodes')
        while not get_aseba_nodes.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_aseba_nodes service not available, waiting again...')
        while True:
            req = GetNodeList.Request()
            resp = get_aseba_nodes.call_async(req)
            rclpy.spin_until_future_complete(self, resp)
            nodes = [node for node in resp.result().nodes
                     if node.name == self.kind]
            if nodes:
                self.get_logger().info(f'Found node {nodes[0]}')
                return (nodes[0].id, nodes[0].running)
            self.get_logger().info(
                f"Waiting for an Aseba node with name {self.kind}")
            time.sleep(1)

    def _aseba(self, topic: str) -> str:
        # if self.namespace:
        #     return f'aseba/{self.namespace}/{topic}'
        return f'aseba/events/{topic}'

    def _ros(self, topic: str) -> str:
        # if self.namespace:
        #     return f'{self.namespace}/{topic}'
        return topic

    def _frame(self, name: str) -> str:
        if self.tf_prefix:
            return f'{self.tf_prefix}/{name}'
        return name

    def init_odometry(self) -> None:
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.odom_frame = self._frame(self.get_or_declare_parameter('odom_frame', 'odom').value)
        self.robot_frame = self._frame(self.get_or_declare_parameter('robot_frame', 'base_link').value)
        self.last_odom_stamp: Optional[rclpy.time.Time] = None
        odom_rate = self.get_or_declare_parameter('odom_max_rate', -1).value
        if odom_rate == 0:
            self.odom_min_period = -1
        else:
            self.odom_min_period = 1.0 / odom_rate
        self.odom_msg = Odometry(header=Header(frame_id=self.odom_frame),
                                 child_frame_id=self.robot_frame)
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.covariance[0] = -1
        self.odom_msg.header.stamp = self.clock.now().to_msg()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self._ros('odom'), 1)

    def init_wheels(self) -> None:
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.left_steps = 0
        self.right_steps = 0
        self.axis_length = self.get_or_declare_parameter('axis_length', self.axis_length).value
        motor_calibration = self.calibration['motor']
        self.motor_speed_conversion = {motor: MotorSpeedConversion(motor_calibration[motor])
                                       for motor in ('left', 'right')}
        self.left_wheel_motor_speed = self.motor_speed_conversion['left'].si_to_aseba
        self.get_logger().info(f'Init left wheel with calibration {motor_calibration["left"]}')
        self.right_wheel_motor_speed = self.motor_speed_conversion['right'].si_to_aseba
        self.get_logger().info(f'Init right wheel with calibration {motor_calibration["right"]}')
        self.update_odom_convariance()
        left_wheel_joint = self.get_or_declare_parameter('left_wheel_joint', 'left_wheel_joint').value
        right_wheel_joint = self.get_or_declare_parameter('right_wheel_joint', 'right_wheel_joint').value
        self.wheel_state_msg = JointState()
        self.wheel_state_msg.name = [self._frame(left_wheel_joint), self._frame(right_wheel_joint)]
        self.wheel_state_pub = self.create_publisher(JointState, self._ros('joint_states'), 1)
        self.aseba_pub = self.create_publisher(AsebaEvent, self._aseba('set_speed'), 1)
        self.create_subscription(
            AsebaEvent, self._aseba('odometry'), self.on_aseba_odometry_event, 1)
        self.create_subscription(Twist, self._ros('cmd_vel'), self.on_cmd_vel, 1)

    def init_proximity(self) -> None:

        self.proximity_sensors: List[ProximitySensor] = []
        proximity_calibration = self.calibration['proximity']
        for name in self.proximity_names:
            pub = self.create_publisher(Range, self._ros(f'proximity/{name}'), 1)
            converter = ProximityConversion(proximity_calibration[name])
            msg = Range(
                header=Header(frame_id=self._frame(f'proximity_{name}_link')),
                radiation_type=Range.INFRARED,
                field_of_view=converter.fov,
                min_range=converter.range_min,
                max_range=converter.range_max)
            self.proximity_sensors.append((pub, msg, converter))

        self.publish_proximity_as_laser = len(self.laser_angles) > 1
        if self.publish_proximity_as_laser:
            prox_sensors_by_name = dict(zip(self.proximity_names, self.proximity_sensors))
            self.laser_publisher = self.create_publisher(
                LaserScan, self._ros('proximity/laser'), 1)
            angle_min = min(self.laser_angles.values())
            angle_max = max(self.laser_angles.values())
            angle_increment = (angle_max - angle_min) / (len(self.laser_angles) - 1)
            self.laser_sensors = [prox_sensors_by_name[name] for name, _
                                  in sorted(self.laser_angles.items(), key=lambda item: item[1])]
            self.laser_indices = [self.proximity_sensors.index(sensor)
                                  for sensor in self.laser_sensors]
            self.laser_msg = LaserScan(
                header=Header(frame_id=self._frame('laser_link')),
                angle_min=angle_min, angle_max=angle_max, angle_increment=angle_increment,
                time_increment=0.0, scan_time=0.0,
                range_min=converter.range_min + self.laser_shift,
                range_max=converter.range_max + self.laser_shift)
        self.create_subscription(
            AsebaEvent, self._aseba('proximity'), self.on_aseba_proximity_event, 1)

    def init(self) -> None:
        pass

    def ready(self, request: std_srvs.srv.Empty.Request, response: std_srvs.srv.Empty.Response
              ) -> std_srvs.srv.Empty.Response:
        return response

    def on_aseba_proximity_event(self, msg: AsebaEvent) -> None:
        for value, (pub, rmsg, converter) in zip(msg.data, self.proximity_sensors):
            rmsg.range = converter.aseba_to_si(value)
            rmsg.header.stamp = self.clock.now().to_msg()
            pub.publish(rmsg)
        if self.publish_proximity_as_laser:
            self.laser_msg.ranges = [
                min(converter.range_max, max(converter.range_min, msg.range)) + self.laser_shift
                for _, msg, converter in self.laser_sensors]
            self.laser_msg.intensities = [float(msg.data[i]) for i in self.laser_indices]
            self.laser_msg.header.stamp = self.clock.now().to_msg()
            self.laser_publisher.publish(self.laser_msg)

    # ======== we send the speed to the aseba running on the robot  ========
    def set_aseba_speed(self, values: List[int]) -> None:
        self.aseba_pub.publish(AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=values))

    # ======== stop the robot safely ========
    def shutdown(self) -> None:
        self.set_aseba_speed([0, 0])

    # ======== processing odometry events received from the robot ========
    def on_aseba_odometry_event(self, data: AsebaEvent) -> None:
        now = rclpy.time.Time.from_msg(data.stamp)
        if self.last_odom_stamp is None:
            self.last_odom_stamp = now
            self.left_steps, self.right_steps = data.data
            return
        dt = (now - self.last_odom_stamp).nanoseconds / 1e9
        if dt <= 0:
            self.get_logger().error(f"Negative time difference {dt} s between odometry messages")
            return
        self.last_odom_stamp = now
        stamp = data.stamp
        left_steps, right_steps = data.data

        left_delta = self.motor_speed_conversion['left'].aseba_to_si(
            delta(left_steps, self.left_steps))
        right_delta = self.motor_speed_conversion['right'].aseba_to_si(
            delta(right_steps, self.right_steps))

        # self.get_logger().info(f"delta {left_delta} {right_delta}")
        self.left_steps = left_steps
        self.right_steps = right_steps

        delta_theta = (right_delta - left_delta) / self.axis_length
        delta_s = 0.5 * (left_delta + right_delta)

        # wheel joint states
        left_wheel_angular_speed = left_delta / self.wheel_radius / dt
        right_wheel_angular_speed = right_delta / self.wheel_radius / dt
        self.left_wheel_angle += left_delta / self.wheel_radius
        self.right_wheel_angle += right_delta / self.wheel_radius

        if delta_theta:
            R = delta_s / delta_theta
            delta_r = (R * sin(delta_theta), R * (1 - cos(delta_theta)))
        else:
            delta_r = (delta_s, 0)
        self.x += delta_r[0] * cos(self.theta) - delta_r[1] * sin(self.theta)
        self.y += delta_r[0] * sin(self.theta) + delta_r[1] * cos(self.theta)
        self.theta += delta_theta

        # We publish odometry, tf, and wheel joint state only at a maximal rate:
        odom_time = rclpy.time.Time.from_msg(self.odom_msg.header.stamp)
        if self.odom_min_period > (now - odom_time).nanoseconds / 1e9:
            return

        self.wheel_state_msg.header.stamp = stamp
        self.wheel_state_msg.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.wheel_state_msg.velocity = [left_wheel_angular_speed, right_wheel_angular_speed]
        self.wheel_state_pub.publish(self.wheel_state_msg)

        # prepare tf from base_link to odom
        quaternion = Quaternion()
        quaternion.z = sin(self.theta / 2.0)
        quaternion.w = cos(self.theta / 2.0)

        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation = quaternion

        self.odom_msg.twist.twist.linear.x = delta_s / dt
        self.odom_msg.twist.twist.angular.z = delta_theta / dt
        self.odom_pub.publish(self.odom_msg)

        pose = self.odom_msg.pose.pose
        translation = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        transform = TransformStamped(
            header=self.odom_msg.header,
            child_frame_id=self.robot_frame,
            transform=Transform(translation=translation, rotation=pose.orientation))
        self.tf_broadcaster.sendTransform(transform)

    def set_linear_angular_speed(self, speed: float, ang_speed: float) -> None:
        left_wheel_speed = speed - ang_speed * 0.5 * self.axis_length
        right_wheel_speed = speed + ang_speed * 0.5 * self.axis_length
        left_motor_speed = self.left_wheel_motor_speed(left_wheel_speed)
        right_motor_speed = self.right_wheel_motor_speed(right_wheel_speed)
        max_motor_speed = max(abs(left_motor_speed), abs(right_motor_speed))
        if max_motor_speed > self.max_aseba_speed:
            left_motor_speed = int(self.max_aseba_speed / max_motor_speed * left_motor_speed)
            right_motor_speed = int(self.max_aseba_speed / max_motor_speed * right_motor_speed)
        self.set_aseba_speed([left_motor_speed, right_motor_speed])

    def on_cmd_vel(self, data: Twist) -> None:
        self.set_linear_angular_speed(data.linear.x, data.angular.z)
