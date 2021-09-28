# type: ignore
import array

import rospy
from asebaros_msgs.msg import Event as AsebaEvent
from sensor_msgs.msg import Imu, Joy, Range, Temperature
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Header, Int8, Int16
from thymio_msgs.msg import Comm, Led, LedGesture, Sound, SystemSound

from .base import BaseDriver

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

DEFAULT_REAL_PROXIMITY_CALIBRATION = dict(
    parameters=[4505.0, 0.0003, 0.0073], kind='power', range_max=0.14, range_min=0.0215, fov=0.3)
)

DEFAULT_SIM_PROXIMITY_CALIBRATION = DEFAULT_REAL_PROXIMITY_CALIBRATION

DEFAULT_REAL_MOTOR_CALIBRATION = dict(
    parameters=[0.001 / SPEED_COEF, 0.0], kind='quadratic', deadband=10
)

DEFAULT_SIM_MOTOR_CALIBRATION = dict(
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

    def default_calibration(self, simulated):
        if simulated:
            proximity = DEFAULT_SIM_PROXIMITY_CALIBRATION
            motor = DEFAULT_SIM_MOTOR_CALIBRATION
        else:
            proximity = DEFAULT_REAL_PROXIMITY_CALIBRATION
            motor = DEFAULT_REAL_MOTOR_CALIBRATION
        return {
            'proximity': {s: proximity for s in self.proximity_names},
            'motor': {s: motor for s in ('left', 'right')}}

    def init(self):
        self.buttons = Joy()
        self.buttons_pub = rospy.Publisher(self._ros('buttons'), Joy, queue_size=1)
        rospy.Subscriber(self._aseba('buttons'), AsebaEvent, self.on_aseba_buttons_event)

        for button in BUTTONS:
            rospy.Subscriber(
                self._aseba('button_{0}'.format(button)), AsebaEvent,
                self.on_aseba_button_event(self._ros('buttons/{0}'.format(button))))

        self.ground_sensors = [{
            'publisher': rospy.Publisher(
                self._ros('ground/{0}'.format(name)), Range, queue_size=1),
            'msg': Range(
                header=Header(
                    frame_id=self._frame('ground_{name}_link'.format(name=name))),
                radiation_type=Range.INFRARED, field_of_view=GROUND_SENSOR_FOV,
                min_range=(GROUND_MIN_RANGE / 1000.0), max_range=(GROUND_MAX_RANGE / 1000.0))
        } for name in GROUND_NAMES]

        self.ground_threshold = rospy.get_param('~ground.threshold', 200)
        rospy.Subscriber(self._aseba('ground'), AsebaEvent, self.on_aseba_ground_event)

        self.imu = Imu(header=Header(frame_id=self.robot_frame))
        # no orientation or angular velocity information
        self.imu.orientation_covariance[0] = -1
        self.imu.angular_velocity_covariance[0] = -1
        # just an accelerometer
        self.imu.linear_acceleration_covariance[0] = 0.07
        self.imu.linear_acceleration_covariance[4] = 0.07
        self.imu.linear_acceleration_covariance[8] = 0.07
        self.imu_publisher = rospy.Publisher(self._ros('imu'), Imu, queue_size=1)
        rospy.Subscriber(
            self._aseba('accelerometer'), AsebaEvent, self.on_aseba_accelerometer_event)

        self.tap_publisher = rospy.Publisher(self._ros('tap'), Empty, queue_size=1)
        rospy.Subscriber(
            self._aseba('tap'), AsebaEvent, self.on_aseba_tap_event)

        self.temperature = Temperature(header=Header(frame_id=self.robot_frame))
        self.temperature.variance = 0.01
        self.temperature_publisher = rospy.Publisher(
            self._ros('temperature'), Temperature, queue_size=1)
        rospy.Subscriber(
            self._aseba('temperature'), AsebaEvent, self.on_aseba_temperature_event)

        self.sound_publisher = rospy.Publisher(self._ros('sound'), Float32, queue_size=1)
        self.sound_threshold_publisher = rospy.Publisher(
            self._aseba('set_sound_threshold'), AsebaEvent, queue_size=1)
        rospy.Subscriber(self._aseba('sound'), AsebaEvent, self.on_aseba_sound_event)
        rospy.Subscriber(self._ros('sound_threshold'), Float32, self.on_sound_threshold)

        self.remote_publisher = rospy.Publisher(self._ros('remote'), Int8, queue_size=1)
        rospy.Subscriber(self._aseba('remote'), AsebaEvent, self.on_aseba_remote_event)

        self.comm_rx_publisher = rospy.Publisher(self._ros('comm/rx'), Comm, queue_size=1)
        self.aseba_enable_comm_publisher = rospy.Publisher(
            self._aseba('enable_comm'), AsebaEvent, queue_size=1)
        self.aseba_set_comm_tx_payload_publisher = rospy.Publisher(
            self._aseba('set_comm_payload'), AsebaEvent, queue_size=1)
        rospy.Subscriber(self._aseba('comm'), AsebaEvent, self.on_aseba_comm_rx_event)
        rospy.Subscriber(self._ros('comm/tx'), Int16, self.on_comm_tx_payload)
        rospy.Subscriber(self._ros('comm/enable'), Bool, self.on_comm_enable)

        # actuators
        for name in BODY_LEDS:
            rospy.Subscriber(self._ros('led/body/{0}'.format(name)), ColorRGBA,
                             self.on_body_led(self._aseba('set_led_{0}'.format(name))))

        rospy.Subscriber(self._ros('led'), Led, self.on_led, 6)
        self.aseba_led_publisher = rospy.Publisher(self._aseba('set_led'), AsebaEvent, queue_size=6)

        rospy.Subscriber(self._ros('led/off'), Empty, self.on_led_off)

        rospy.Subscriber(self._ros('led/gesture'), LedGesture, self.on_led_gesture)
        self.aseba_led_gesture_publisher = rospy.Publisher(
            self._aseba('set_led_gesture'), AsebaEvent, queue_size=6)
        rospy.Subscriber(
            self._ros('led/gesture/circle'), Float32, self.on_led_gesture_circle)
        rospy.Subscriber(
            self._ros('led/gesture/off'), Empty, self.on_led_gesture_off)
        rospy.Subscriber(
            self._ros('led/gesture/blink'), Float32, self.on_led_gesture_blink)
        rospy.Subscriber(
            self._ros('led/gesture/kit'), Float32, self.on_led_gesture_kit)
        rospy.Subscriber(
            self._ros('led/gesture/alive'), Empty, self.on_led_gesture_alive)

        rospy.Subscriber(self._ros('sound/play'), Sound, self.on_sound_play)
        rospy.Subscriber(
            self._ros('sound/play/system'), SystemSound, self.on_system_sound_play)
        self.aseba_play_sound_publisher = rospy.Publisher(
            self._aseba('play_sound'), AsebaEvent, queue_size=1)
        self.aseba_play_system_sound_publisher = rospy.Publisher(
            self._aseba('play_system_sound'), AsebaEvent, queue_size=1)

        rospy.Subscriber(self._ros('alarm'), Bool, self.on_alarm)
        self.alarm_timer = None

        rospy.Subscriber(self._ros('shutdown'), Empty, self.on_shutdown_msg)
        self.aseba_shutdown_publisher = rospy.Publisher(
            self._aseba('shutdown'), AsebaEvent, queue_size=1)

    def on_aseba_comm_rx_event(self, msg):
        rmsg = Comm()
        rmsg.value = msg.data[0]
        # REVIEW: Why must I cast to a list???
        rmsg.payloads = list(msg.data[1:8])
        rmsg.intensities = list(msg.data[8:])
        self.comm_rx_publisher.publish(rmsg)

    def on_comm_enable(self, msg):
        msg = AsebaEvent(stamp=rospy.Time.now(), source=0, data=[msg.data])
        self.aseba_enable_comm_publisher.publish(msg)

    def on_comm_tx_payload(self, msg):
        msg = AsebaEvent(stamp=rospy.Time.now(), source=0, data=[msg.data])
        self.aseba_set_comm_tx_payload_publisher.publish(msg)

    def on_shutdown_msg(self, msg):
        self.aseba_shutdown_publisher.publish(
            AsebaEvent(stamp=rospy.Time.now(), source=0, data=[]))

    def play_system_sound(self, sound):
        self.aseba_play_system_sound_publisher.publish(
            AsebaEvent(stamp=rospy.Time.now(), source=0, data=[sound]))

    def on_alarm(self, msg):
        if msg.data and not self.alarm_timer:
            self.alarm_timer = rospy.Timer(
                rospy.Duration(3.0), lambda evt: self.play_system_sound(2))
        if not msg.data and self.alarm_timer:
            self.alarm_timer.shutdown()
            self.alarm_timer = None

    def on_sound_play(self, msg):
        freq = max(1, int(msg.frequency))
        duration = max(1, int(msg.duration * 60))
        self.aseba_play_sound_publisher.publish(
            AsebaEvent(stamp=rospy.Time.now(), source=0, data=[freq, duration]))

    def on_system_sound_play(self, msg):
        self.play_system_sound(msg.sound)

    def set_led_gesture(self, gesture, leds, wave, period,
                        length, mirror, mask):
        period = max(-32678, min(32678, int(period * 1000)))
        data = [gesture, leds, wave, period, length, mirror] + mask[:8]
        data += [1] * (14 - len(data))
        self.aseba_led_gesture_publisher.publish(
            AsebaEvent(stamp=rospy.Time.now(), source=0, data=data))

    def on_led_gesture(self, msg):
        self.set_led_gesture(msg.gesture, msg.leds, msg.wave,
                             msg.period, msg.length, msg.mirror, msg.mask)

    def on_led_gesture_off(self, msg):
        self.set_led_gesture(LedGesture.OFF, 0, 0, 0, 0, 0, [])

    def on_led_gesture_circle(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 8, 0, [])

    def on_led_gesture_blink(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.HARMONIC, msg.data, 1, 0, [])

    def on_led_gesture_kit(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.PROXIMITY,
                             LedGesture.HARMONIC, msg.data, 12, 11, [1, 1, 1, 1, 1, 1, 0, 0])

    def on_led_gesture_alive(self, msg):
        self.set_led_gesture(LedGesture.WAVE, LedGesture.CIRCLE,
                             LedGesture.RECT, 3.0, 24, 0, [])

    def on_led_off(self, msg):
        for i in LED_NUMBER.keys():
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=rospy.Time.now(), source=0, data=[i] + 8 * [0]))
            # sleep to avoid that aseba or ros do not process all messages.
            # could be improved by having 6 separate aseba topics where to send
            # messages
            rospy.sleep(0.005)

    def on_led(self, msg):
        i = msg.id
        num = LED_NUMBER.get(i, 0)
        if num <= len(msg.values):
            data = [i] + [int(32 * v) for v in msg.values[:8]]
            data += [0] * (9 - len(data))
            self.aseba_led_publisher.publish(
                AsebaEvent(stamp=rospy.Time.now(), source=0, data=data))

    def on_body_led(self, topic):
        publisher = rospy.Publisher(topic, AsebaEvent, queue_size=1)

        def callback(msg):
            r = int(msg.r * 32)
            g = int(msg.g * 32)
            b = int(msg.b * 32)
            aseba_msg = AsebaEvent(stamp=rospy.Time.now(), source=0, data=[r, g, b])
            publisher.publish(aseba_msg)
        return callback

    def on_aseba_remote_event(self, msg):
        self.remote_publisher.publish(Int8(data=msg.data[1]))

    def on_sound_threshold(self, msg):
        value = msg * 255
        if value < 0:
            value = 1
        if value > 255:
            value = 0
        self.sound_threshold_publisher.publish(
            AsebaEvent(stamp=rospy.Time.now(), source=0, data=[value]))

    def on_aseba_sound_event(self, msg):
        self.sound_publisher.publish(Float32(data=msg.data[0] / 255.0))

    def on_aseba_tap_event(self, msg):
        self.tap_publisher.publish(Empty())

    def on_aseba_temperature_event(self, msg):
        self.temperature.temperature = msg.data[0] / 10.0
        self.temperature_publisher.publish(self.temperature)

    # TODO check how it's implemented in the firmware.
    def on_aseba_accelerometer_event(self, msg):
        self.imu.linear_acceleration.x = msg.data[1] / 23.0 * 9.81
        self.imu.linear_acceleration.y = -msg.data[0] / 23.0 * 9.81
        self.imu.linear_acceleration.z = msg.data[2] / 23.0 * 9.81
        self.imu.header.stamp = rospy.Time.now()
        self.imu_publisher.publish(self.imu)

    def on_aseba_ground_event(self, msg):
        data = msg.data
        # ir_threshold = self.get_parameter("ground.threshold").value
        ir_threshold = self.ground_threshold

        for sensor, value in zip(self.ground_sensors, data):
            #  HACK(Jerome): Galactic does not like +/- inf
            # sensor['msg'].range = float('inf') if (
            #     value < ir_threshold) else -float('inf')
            sensor['msg'].range = 1.0 if (
                value < ir_threshold) else 0.0
            sensor['msg'].header.stamp = rospy.Time.now()
            sensor['publisher'].publish(sensor['msg'])

    def on_aseba_button_event(self, topic):
        publisher = rospy.Publisher(topic, Bool, queue_size=1)

        def callback(msg):
            bool_msg = Bool(data=bool(msg.data[0]))
            publisher.publish(bool_msg)
        return callback

    def on_aseba_buttons_event(self, data):
        self.buttons.header.stamp = rospy.Time.now()
        # data.data is array('h'), we need array('i')
        self.buttons.buttons = array.array('i', data.data)
        self.buttons_pub.publish(self.buttons)
