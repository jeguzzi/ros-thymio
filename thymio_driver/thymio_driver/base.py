# type: ignore
import time
from math import copysign, cos, sin, sqrt

import rospy
import std_srvs.srv
# from asebaros_msgs.msg import AsebaEvent
from asebaros_msgs.msg import Event as AsebaEvent
from asebaros_msgs.srv import GetNodeList, LoadScript
from geometry_msgs.msg import (Quaternion, Transform, TransformStamped, Twist,
                               Vector3)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan, Range
from std_msgs.msg import Header
from tf2_ros.transform_broadcaster import TransformBroadcaster


def sign(x):
    return int(copysign(1, x))


class MotorSpeedConversion(object):

    def __init__(self, calibration):
        if calibration['kind'] != 'quadratic':
            raise ValueError("Unknown calibration {0}".format(calibration['kind']))
        self.parameters = calibration['parameters']
        self.deadband = calibration['deadband']

    @property
    def parameters(self):
        return [self.q0, self.q1]

    @parameters.setter
    def parameters(self, value):
        self.q0, self.q1 = value[:2]

    def aseba_to_si(self, value):
        # if abs(value) < self.deadband:
        #     return 0.0
        return self.q0 * value + self.q1 * value * abs(value)

    def si_to_aseba(self, value):
        if self.q1 == 0:
            return int(round(value / self.q0))
        return int(round(0.5 * sign(value) *
                   (- self.q0 + sqrt(self.q0 ** 2 + 4 * self.q1 * abs(value))) / self.q1))


class ProximityConversion(object):

    def __init__(self, calibration):
        if calibration['kind'] != 'power':
            raise ValueError("Unknown calibration {0}".format(calibration['kind']))
        self.parameters = calibration['parameters']
        self.range_max = calibration['range_max']
        self.range_min = calibration['range_min']
        self.fov = calibration['fov']

    @property
    def parameters(self):
        return [self.max_value, self.x0, self.c]

    @parameters.setter
    def parameters(self, value):
        self.max_value, self.x0, self.c = value[:3]

    def aseba_to_si(self, value):
        if value == 0:
            return float('inf')
        if value >= self.max_value:
            return -float('inf')
        dist = self.x0 + sqrt((self.x0 ** 2 - self.c) * (1 - self.max_value / value))
        return max(self.range_min, min(dist, self.range_max))


_m = 2 ** 15


def delta(v1, v0):
    diff = v1 - v0
    if diff > _m:
        diff -= 2 * _m
    elif diff < -_m:
        diff += 2 * _m
    return diff


class BaseDriver(object):

    def __init__(self, namespace='', standalone=True):
        # TODO(ROS1): check if we don't need the namespace enaymbe
        # super(BaseDriver, self).__init__('driver', namespace=namespace)
        rospy.loginfo("Create %s with namespace %s, standalone %s",
                      type(self), namespace, standalone)
        self.namespace = namespace
        self.node_id = None
        if standalone:
            # TODO(Jerome): better semantic
            if not self.wait_for_aseba_node():
                self.load_script()
        # self.clock = self.get_clock()
        if namespace:
            self.tf_prefix = namespace
        else:
            self.tf_prefix = rospy.get_param('~tf_prefix', '')
        self.init_odometry()
        self.init_wheels()
        self.init_proximity()
        # TODO(ROS1): Dynamic reconfig callback
        # self.add_on_set_parameters_callback(self.param_callback)
        self.init()
        # REVIEW: not implemented yet in ROS2
        # self.on_shutdown(self.shutdown)
        rospy.Service(self._ros('is_ready'), std_srvs.srv.Empty, self.ready)
        rospy.loginfo("%s is ready", self.kind)

    # TODO(ROS1): Dynamic reconfig callback
    # def param_callback(self, parameters: List[rclpy.parameter.Parameter]
    #                    ) -> SetParametersResult:
    #     # TODO(Jerome): move motor deadband
    #     for param in parameters:
    #         rospy.logdebug(f'will set param {param.name} to {param.value}')
    #         for motor in ('left', 'right'):
    #             if f'motor_calibration.{motor}' in param.name:
    #                 # TODO: enforce bounds
    #                 conversion = self.motor_speed_conversion[motor]
    #                 if 'parameters' in param.name:
    #                     conversion.parameters = param.value
    #                 if 'deadband' in param.name:
    #                     conversion.deadband = param.value
    #                     rospy.logdebug(
    #                         f"Changed {motor} motor deadband to {param.value}")
    #                     self.update_odom_convariance()
    #     response = SetParametersResult(successful=True)
    #     return response

    def update_odom_convariance(self):
        si_band = sum(c.aseba_to_si(c.deadband) for c in self.motor_speed_conversion.values()) / 2
        self.odom_msg.twist.covariance[0] = speed_cov = 0.5 * si_band ** 2
        self.odom_msg.twist.covariance[-1] = speed_cov / (self.axis_length ** 2)

    def load_script(self):
        # default_script = os.path.join(get_package_share_directory(
        #     'thymio_driver'), 'aseba', 'thymio_ros.aesl')
        default_script = ''
        script_path = rospy.get_param('~script', default_script)
        if not script_path:
            rospy.logwarn('Script not provided!')
            return

        rospy.loginfo('Try to load script at %s', format(script_path))
        while True:
            try:
                rospy.wait_for_service('aseba/load_script', timeout=1.0)
                break
            except rospy.ROSException:
                rospy.loginfo('load_script service not available, waiting again...')
        load_script = rospy.ServiceProxy('aseba/load_script', LoadScript)
        if self.node_id is not None:
            load_script(file_name=script_path, node_ids=[self.node_id])

    def wait_for_aseba_node(self):
        while True:
            try:
                rospy.wait_for_service('aseba/get_nodes', timeout=1.0)
                break
            except rospy.ROSException:
                rospy.loginfo('get_aseba_nodes service not available, waiting again...')
        get_aseba_nodes = rospy.ServiceProxy('aseba/get_nodes', GetNodeList)
        while True:
            resp = get_aseba_nodes()
            nodes = [node for node in resp.nodes if node.name == self.kind]
            if nodes:
                rospy.loginfo('Found Thymio %s', {nodes[0]})
                self.node_id = nodes[0].id
                return nodes[0].running
            rospy.loginfo('Waiting for a node of kind %s ...', self.kind)
            time.sleep(1)

    def _aseba(self, topic):
        if self.namespace:
            return '{0}/aseba/events/{1}'.format(self.namespace, topic)
        return 'aseba/events/{0}'.format(topic)

    def _ros(self, topic):
        if self.namespace:
            return '{0}/{1}'.format(self.namespace, topic)
        return topic

    def _frame(self, name):
        if self.tf_prefix:
            return '{0}/{1}'.format(self.tf_prefix, name)
        return name

    def init_odometry(self):
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.odom_frame = self._frame(rospy.get_param('~odom_frame', 'odom'))
        self.robot_frame = self._frame(rospy.get_param('~robot_frame', 'base_link'))
        self.last_odom_stamp = None
        odom_rate = rospy.get_param('~odom_max_rate', -1)
        if odom_rate == 0:
            self.odom_min_period = -1
        else:
            self.odom_min_period = 1.0 / odom_rate
        self.odom_msg = Odometry(header=Header(frame_id=self.odom_frame),
                                 child_frame_id=self.robot_frame)
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.covariance[0] = -1
        self.odom_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster = TransformBroadcaster()
        self.odom_pub = rospy.Publisher(self._ros('odom'), Odometry, queue_size=1)

    def load_calibration(self, group, name, default):
        return {key: rospy.get_param('~{0}/{1}/{2}'.format(group, name, key), value)
                for key, value in default.items()}

    def init_wheels(self):
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.left_steps = 0
        self.right_steps = 0
        self.axis_length = rospy.get_param('~axis_length', self.axis_length)
        motor_calibration = {
            motor: self.load_calibration('motor_calibration', motor, self.motor_calibration)
            for motor in ('left', 'right')}
        self.motor_speed_conversion = {motor: MotorSpeedConversion(motor_calibration[motor])
                                       for motor in ('left', 'right')}
        self.left_wheel_motor_speed = self.motor_speed_conversion['left'].si_to_aseba
        rospy.loginfo('Init left wheel with calibration %s', motor_calibration["left"])
        self.right_wheel_motor_speed = self.motor_speed_conversion['right'].si_to_aseba
        rospy.loginfo('Init right wheel with calibration %s', motor_calibration["right"])
        self.update_odom_convariance()
        left_wheel_joint = rospy.get_param('~left_wheel_joint', 'left_wheel_joint')
        right_wheel_joint = rospy.get_param('~right_wheel_joint', 'right_wheel_joint')
        self.wheel_state_msg = JointState()
        self.wheel_state_msg.name = [self._frame(left_wheel_joint), self._frame(right_wheel_joint)]
        self.wheel_state_pub = rospy.Publisher(self._ros('joint_states'), JointState, queue_size=1)
        self.aseba_pub = rospy.Publisher(self._aseba('set_speed'), AsebaEvent, queue_size=1)
        rospy.Subscriber(
            self._aseba('odometry'), AsebaEvent, self.on_aseba_odometry_event)
        rospy.Subscriber(self._ros('cmd_vel'), Twist, self.on_cmd_vel)

    def init_proximity(self):

        self.proximity_sensors = []

        for name in self.proximity_names:
            pub = rospy.Publisher(self._ros('proximity/{0}'.format(name)), Range, queue_size=1)
            converter = ProximityConversion(
                self.load_calibration('proximity_calibration', name,
                                      default=self.proximity_calibration))
            msg = Range(
                header=Header(frame_id=self._frame('proximity_{0}_link'.format(name))),
                radiation_type=Range.INFRARED,
                field_of_view=converter.fov,
                min_range=converter.range_min,
                max_range=converter.range_max)
            self.proximity_sensors.append((pub, msg, converter))

        self.publish_proximity_as_laser = len(self.laser_angles) > 1
        if self.publish_proximity_as_laser:
            prox_sensors_by_name = dict(zip(self.proximity_names, self.proximity_sensors))
            self.laser_publisher = rospy.Publisher(
                self._ros('proximity/laser'), LaserScan, queue_size=1)
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
        rospy.Subscriber(
            self._aseba('proximity'), AsebaEvent, self.on_aseba_proximity_event)

    def init(self):
        pass

    def ready(self, req):
        return std_srvs.srv.EmptyResponse()

    def on_aseba_proximity_event(self, msg):
        for value, (pub, rmsg, converter) in zip(msg.data, self.proximity_sensors):
            rmsg.range = converter.aseba_to_si(value)
            rmsg.header.stamp = rospy.Time.now()
            pub.publish(rmsg)
        if self.publish_proximity_as_laser:
            self.laser_msg.ranges = [
                min(converter.range_max, max(converter.range_min, rmsg.range)) + self.laser_shift
                for _, rmsg, converter in self.laser_sensors]
            self.laser_msg.intensities = [float(msg.data[i]) for i in self.laser_indices]
            self.laser_msg.header.stamp = rospy.Time.now()
            self.laser_publisher.publish(self.laser_msg)

    # ======== we send the speed to the aseba running on the robot  ========
    def set_aseba_speed(self, values):
        self.aseba_pub.publish(AsebaEvent(stamp=rospy.Time.now(), source=0, data=values))

    # ======== stop the robot safely ========
    def shutdown(self):
        self.set_aseba_speed([0, 0])

    # ======== processing odometry events received from the robot ========
    def on_aseba_odometry_event(self, data):
        now = data.stamp
        if self.last_odom_stamp is None:
            self.last_odom_stamp = now
            self.left_steps, self.right_steps = data.data
            return
        dt = (now - self.last_odom_stamp).to_sec()
        if dt <= 0:
            rospy.logerror("Negative time difference {%.3f} s between odometry messages", dt)
            return
        self.last_odom_stamp = now
        stamp = data.stamp
        left_steps, right_steps = data.data

        left_delta = self.motor_speed_conversion['left'].aseba_to_si(
            delta(left_steps, self.left_steps))
        right_delta = self.motor_speed_conversion['right'].aseba_to_si(
            delta(right_steps, self.right_steps))

        # rospy.loginfo(f"delta {left_delta} {right_delta}")
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
        odom_time = self.odom_msg.header.stamp
        if self.odom_min_period > (now - odom_time).to_sec():
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

    def set_linear_angular_speed(self, speed, ang_speed):
        left_wheel_speed = speed - ang_speed * 0.5 * self.axis_length
        right_wheel_speed = speed + ang_speed * 0.5 * self.axis_length
        left_motor_speed = self.left_wheel_motor_speed(left_wheel_speed)
        right_motor_speed = self.right_wheel_motor_speed(right_wheel_speed)
        max_motor_speed = max(abs(left_motor_speed), abs(right_motor_speed))
        if max_motor_speed > self.max_aseba_speed:
            left_motor_speed = int(self.max_aseba_speed / max_motor_speed * left_motor_speed)
            right_motor_speed = int(self.max_aseba_speed / max_motor_speed * right_motor_speed)
        self.set_aseba_speed([left_motor_speed, right_motor_speed])

    def on_cmd_vel(self, data):
        self.set_linear_angular_speed(data.linear.x, data.angular.z)
