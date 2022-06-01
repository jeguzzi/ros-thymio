import array
import time
from typing import Any, Callable, List, Optional

try:
    from typing import TypedDict
except ImportError:
    from typing_extensions import TypedDict

import rclpy
import rclpy.duration
import rclpy.publisher
import rclpy.timer
from asebaros_msgs.msg import Event as AsebaEvent
from sensor_msgs.msg import Imu, Joy, Range, Temperature
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Header, Int8, Int16
from thymio_msgs.msg import Comm, Led, LedGesture, Sound, SystemSound

from .base import BaseDriver, Calibration, ProximityCalibration, MotorCalibration

GroundSensor = TypedDict('GroundSensor', {'publisher': rclpy.publisher.Publisher, 'msg': Range})

BASE_WIDTH = 91.5     # millimeters
SPEED_COEF = 2.93     # 1mm/sec corresponds to X units of real thymio speed
WHEEL_RADIUS = 0.022   # meters
GROUND_MIN_RANGE = 9     # millimeters
GROUND_MAX_RANGE = 30     # millimeters
GROUND_SENSOR_FOV = 0.3  # radians

BUTTONS = ['backward', 'forward', 'center', 'right', 'left']
GROUND_NAMES = ['left', 'right']
BODY_LEDS = ['bottom_left', 'bottom_right', 'top']
LED_NUMBER = {Led.CIRCLE: 8, Led.PROXIMITY: 8, Led.GROUND: 2,
              Led.REMOTE: 1, Led.BUTTONS: 4, Led.TEMPERATURE: 2, Led.MICROPHONE: 1}


DEFAULT_REAL_PROXIMITY_CALIBRATION = ProximityCalibration(
    parameters=[4505.0, 0.0003, 0.0073], kind='power', range_max=0.14, range_min=0.0215, fov=0.3)

DEFAULT_SIM_PROXIMITY_CALIBRATION = DEFAULT_REAL_PROXIMITY_CALIBRATION

DEFAULT_REAL_MOTOR_CALIBRATION = MotorCalibration(
    parameters=[0.001 / SPEED_COEF, 0.0], kind='quadratic', deadband=10
)

DEFAULT_SIM_MOTOR_CALIBRATION = MotorCalibration(
    parameters=[0.000332, 0.0], kind='quadratic', deadband=0
)


class ThymioDriver(BaseDriver):

    kind = 'thymio-II'
    axis_length = BASE_WIDTH / 1000
    wheel_radius = WHEEL_RADIUS
    max_aseba_speed = 500
    proximity_names = ['left', 'center_left', 'center', 'center_right', 'right',
                       'rear_left', 'rear_right']
    laser_angles = {'left': 0.64, 'center_left': 0.32, 'center': 0,
                    'center_right': -0.32, 'right': -0.64}
    laser_shift = 0.08

    def default_calibration(self, simulated: bool = False) -> Calibration:
        if simulated:
            proximity = DEFAULT_SIM_PROXIMITY_CALIBRATION
            motor = DEFAULT_SIM_MOTOR_CALIBRATION
        else:
            proximity = DEFAULT_REAL_PROXIMITY_CALIBRATION
            motor = DEFAULT_REAL_MOTOR_CALIBRATION
        return {
            'proximity': {s: proximity for s in self.proximity_names},
            'motor': {s: motor for s in ('left', 'right')}}

    def init(self) -> None:

        self.buttons = Joy()
        self.buttons_pub = self.create_publisher(Joy, self._ros('buttons'), 1)
        self.create_subscription(AsebaEvent, self._aseba('buttons'),
                                 self.on_aseba_buttons_event, 1)

        for button in BUTTONS:
            self.create_subscription(
                AsebaEvent, self._aseba(f'button_{button}'),
                self.on_aseba_button_event(self._ros(f'buttons/{button}')), 1)

        self.ground_sensors: List[GroundSensor] = [{
            'publisher': self.create_publisher(Range, self._ros(f'ground/{name}'), 1),
            'msg': Range(
                header=Header(
                    frame_id=self._frame('ground_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED, field_of_view=GROUND_SENSOR_FOV,
                min_range=(GROUND_MIN_RANGE / 1000.0), max_range=(GROUND_MAX_RANGE / 1000.0))
        } for name in GROUND_NAMES]

        self.ground_threshold = self.get_or_declare_parameter('ground.threshold', 200).value
        self.create_subscription(AsebaEvent, self._aseba('ground'), self.on_aseba_ground_event, 1)

        self.imu = Imu(header=Header(frame_id=self.robot_frame))
        # no orientation or angular velocity information
        self.imu.orientation_covariance[0] = -1
        self.imu.angular_velocity_covariance[0] = -1
        # just an accelerometer
        self.imu.linear_acceleration_covariance[0] = 0.07
        self.imu.linear_acceleration_covariance[4] = 0.07
        self.imu.linear_acceleration_covariance[8] = 0.07
        self.imu_publisher = self.create_publisher(Imu, self._ros('imu'), 1)
        self.create_subscription(
            AsebaEvent, self._aseba('accelerometer'), self.on_aseba_accelerometer_event, 1)

        self.tap_publisher = self.create_publisher(Empty, self._ros('tap'), 1)
        self.create_subscription(
            AsebaEvent, self._aseba('tap'), self.on_aseba_tap_event, 1)

        self.temperature = Temperature(header=Header(frame_id=self.robot_frame))
        self.temperature.variance = 0.01
        self.temperature_publisher = self.create_publisher(
            Temperature, self._ros('temperature'), 1)
        self.create_subscription(
            AsebaEvent, self._aseba('temperature'), self.on_aseba_temperature_event, 1)

        self.sound_publisher = self.create_publisher(Float32, self._ros('sound'), 1)
        self.sound_threshold_publisher = self.create_publisher(
            AsebaEvent, self._aseba('set_sound_threshold'), 1)
        self.create_subscription(AsebaEvent, self._aseba('sound'), self.on_aseba_sound_event, 1)
        self.create_subscription(Float32, self._ros('sound_threshold'), self.on_sound_threshold, 1)

        self.remote_publisher = self.create_publisher(Int8, self._ros('remote'), 1)
        self.create_subscription(AsebaEvent, self._aseba('remote'), self.on_aseba_remote_event, 1)

        self.comm_rx_publisher = self.create_publisher(Comm, self._ros('comm/rx'), 1)
        self.aseba_enable_comm_publisher = self.create_publisher(
            AsebaEvent, self._aseba('enable_comm'), 1)
        self.aseba_set_comm_tx_payload_publisher = self.create_publisher(
            AsebaEvent, self._aseba('set_comm_payload'), 1)
        self.create_subscription(AsebaEvent, self._aseba('comm'), self.on_aseba_comm_rx_event, 1)
        self.create_subscription(Int16, self._ros('comm/tx'), self.on_comm_tx_payload, 1)
        self.create_subscription(Bool, self._ros('comm/enable'), self.on_comm_enable, 1)

        # actuators
        for name in BODY_LEDS:
            self.create_subscription(ColorRGBA, self._ros(f'led/body/{name}'),
                                     self.on_body_led(self._aseba(f'set_led_{name}')), 1)

        self.create_subscription(Led, self._ros('led'), self.on_led, 6)
        self.aseba_led_publisher = self.create_publisher(AsebaEvent, self._aseba('set_led'), 6)

        self.create_subscription(Empty, self._ros('led/off'), self.on_led_off, 1)

        self.create_subscription(LedGesture, self._ros('led/gesture'), self.on_led_gesture, 1)
        self.aseba_led_gesture_publisher = self.create_publisher(
            AsebaEvent, self._aseba('set_led_gesture'), 6)
        self.create_subscription(
            Float32, self._ros('led/gesture/circle'), self.on_led_gesture_circle, 1)
        self.create_subscription(
            Empty, self._ros('led/gesture/off'), self.on_led_gesture_off, 1)
        self.create_subscription(
            Float32, self._ros('led/gesture/blink'), self.on_led_gesture_blink, 1)
        self.create_subscription(
            Float32, self._ros('led/gesture/kit'), self.on_led_gesture_kit, 1)
        self.create_subscription(
            Empty, self._ros('led/gesture/alive'), self.on_led_gesture_alive, 1)

        self.create_subscription(Sound, self._ros('sound/play'), self.on_sound_play, 1)
        self.create_subscription(
            SystemSound, self._ros('sound/play/system'), self.on_system_sound_play, 1)
        self.aseba_play_sound_publisher = self.create_publisher(
            AsebaEvent, self._aseba('play_sound'), 1)
        self.aseba_play_system_sound_publisher = self.create_publisher(
            AsebaEvent, self._aseba('play_system_sound'), 1)

        self.create_subscription(Bool, self._ros('alarm'), self.on_alarm, 1)
        self.alarm_timer: Optional[rclpy.timer.Timer] = None

        self.create_subscription(Empty, self._ros('shutdown'), self.on_shutdown_msg, 1)
        self.aseba_shutdown_publisher = self.create_publisher(
            AsebaEvent, self._aseba('shutdown'), 1)

    def on_aseba_comm_rx_event(self, msg: AsebaEvent) -> None:
        rmsg = Comm()
        rmsg.value = msg.data[0]
        # REVIEW: Why must I cast to a list???
        rmsg.payloads = list(msg.data[1:8])
        rmsg.intensities = list(msg.data[8:])
        self.comm_rx_publisher.publish(rmsg)

    def on_comm_enable(self, msg: Bool) -> None:
        out_msg = AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[msg.data])
        self.aseba_enable_comm_publisher.publish(out_msg)

    def on_comm_tx_payload(self, msg: Int16) -> None:
        out_msg = AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[msg.data])
        self.aseba_set_comm_tx_payload_publisher.publish(out_msg)

    def on_shutdown_msg(self, msg: Empty) -> None:
        self.aseba_shutdown_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[]))

    def play_system_sound(self, sound: int) -> None:
        self.aseba_play_system_sound_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[sound]))

    def on_alarm(self, msg: Bool) -> None:
        if msg.data and not self.alarm_timer:
            self.alarm_timer = self.create_timer(3.0, lambda: self.play_system_sound(2))
        if not msg.data and self.alarm_timer:
            self.alarm_timer.destroy()
            self.alarm_timer = None

    def on_sound_play(self, msg: Sound) -> None:
        freq = max(1, int(msg.frequency))
        duration = rclpy.duration.Duration.from_msg(msg.duration).nanoseconds / 1e9
        duration = max(1, int(duration * 60))
        self.aseba_play_sound_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[freq, duration]))

    def on_system_sound_play(self, msg: SystemSound) -> None:
        self.play_system_sound(msg.sound)

    def set_led_gesture(self, gesture: int, leds: int, wave: int, period: float,
                        length: int, mirror: int, mask: List[int]) -> None:
        period = max(-32678, min(32678, int(period * 1000)))
        data = [gesture, leds, wave, period, length, mirror] + mask[:8]
        data += [1] * (14 - len(data))
        self.aseba_led_gesture_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=data))

    def on_led_gesture(self, msg: LedGesture) -> None:
        self.set_led_gesture(msg.gesture, msg.leds, msg.wave,
                             msg.period, msg.length, msg.mirror, msg.mask)

    def on_led_gesture_off(self, msg: Empty) -> None:
        self.set_led_gesture(LedGesture.OFF, 0, 0, 0, 0, 0, [])

    def on_led_gesture_circle(self, msg: Float32) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 8, 0, [])

    def on_led_gesture_blink(self, msg: Float32) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 1, 0, [])

    def on_led_gesture_kit(self, msg: Float32) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.PROXIMITY,
                             LedGesture.HARMONIC, msg.data, 12, 11, [1, 1, 1, 1, 1, 1, 0, 0])

    def on_led_gesture_alive(self, msg: Empty) -> None:
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.RECT, 3.0, 24, 0, [])

    def on_led_off(self, msg: Empty) -> None:
        for i in LED_NUMBER.keys():
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[i] + 8 * [0]))
            # sleep to avoid that aseba or ros do not process all messages.
            # could be improved by having 6 separate aseba topics where to send
            # messages
            time.sleep(0.005)

    def on_led(self, msg: Led) -> None:
        i = msg.id
        num = LED_NUMBER.get(i, 0)
        if num <= len(msg.values):
            data = [i] + [int(32 * v) for v in msg.values[:8]]
            data += [0] * (9 - len(data))
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=data))

    def on_body_led(self, topic: str) -> Callable[[ColorRGBA], None]:
        publisher = self.create_publisher(AsebaEvent, topic, 1)

        def callback(msg: ColorRGBA) -> None:
            r = int(msg.r * 32)
            g = int(msg.g * 32)
            b = int(msg.b * 32)
            aseba_msg = AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[r, g, b])
            publisher.publish(aseba_msg)
        return callback

    def on_aseba_remote_event(self, msg: AsebaEvent) -> None:
        self.remote_publisher.publish(Int8(data=msg.data[1]))

    def on_sound_threshold(self, msg: AsebaEvent) -> None:
        if not msg.data:
            self.get_logger().warn("on_sound_threshold: Aseba event message has not enough data")
            return
        value = msg.data[0] * 255
        if value < 0:
            value = 1
        if value > 255:
            value = 0
        self.sound_threshold_publisher.publish(
            AsebaEvent(stamp=self.clock.now().to_msg(), source=0, data=[value]))

    def on_aseba_sound_event(self, msg: AsebaEvent) -> None:
        self.sound_publisher.publish(Float32(data=msg.data[0] / 255.0))

    def on_aseba_tap_event(self, msg: AsebaEvent) -> None:
        self.tap_publisher.publish(Empty())

    def on_aseba_temperature_event(self, msg: AsebaEvent) -> None:
        self.temperature.temperature = msg.data[0] / 10.0
        self.temperature_publisher.publish(self.temperature)

    # TODO check how it's implemented in the firmware.
    def on_aseba_accelerometer_event(self, msg: AsebaEvent) -> None:
        self.imu.linear_acceleration.x = msg.data[1] / 23.0 * 9.81
        self.imu.linear_acceleration.y = -msg.data[0] / 23.0 * 9.81
        self.imu.linear_acceleration.z = msg.data[2] / 23.0 * 9.81
        self.imu.header.stamp = self.clock.now().to_msg()
        self.imu_publisher.publish(self.imu)

    def on_aseba_ground_event(self, msg: AsebaEvent) -> None:
        data = msg.data
        # ir_threshold = self.get_parameter("ground.threshold").value
        ir_threshold = self.ground_threshold

        for sensor, value in zip(self.ground_sensors, data):
            #  HACK(Jerome): Galactic does not like +/- inf
            # sensor['msg'].range = float('inf') if (
            #     value < ir_threshold) else -float('inf')
            sensor['msg'].range = 1.0 if (
                value < ir_threshold) else 0.0
            sensor['msg'].header.stamp = self.clock.now().to_msg()
            sensor['publisher'].publish(sensor['msg'])

    def on_aseba_button_event(self, topic: str) -> Callable[[AsebaEvent], None]:
        publisher = self.create_publisher(Bool, topic, 1)

        def callback(msg: AsebaEvent) -> None:
            bool_msg = Bool(data=bool(msg.data[0]))
            publisher.publish(bool_msg)
        return callback

    def on_aseba_buttons_event(self, data: AsebaEvent) -> None:
        self.buttons.header.stamp = self.clock.now().to_msg()
        # data.data is array('h'), we need array('i')
        self.buttons.buttons = array.array('i', data.data)
        self.buttons_pub.publish(self.buttons)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    thymio_driver = ThymioDriver()
    try:
        rclpy.spin(thymio_driver)
    except KeyboardInterrupt:
        pass
    thymio_driver.destroy_node()
    rclpy.shutdown()
