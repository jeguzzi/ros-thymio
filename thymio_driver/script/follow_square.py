#!/usr/bin/env python
# type: ignore
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class FollowSquare(object):

    def __init__(self):
        rospy.init_node("do_square")
        rospy.wait_for_service('is_ready')
        speed = rospy.get_param('~speed', 0.05)
        axis_length = rospy.get_param('~axis_length', 0.0935)
        side_length = rospy.get_param('~side', 0.6)
        rospy.loginfo('Init open loop square')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.update_odom)
        self.pose = (0.0, 0.0, 0.0)
        self.sleep(1)
        rospy.loginfo(
            'Start square with speed {0:.3f} m/s and side {1:.3f} m'.format(speed, side_length))
        for _ in range(4):
            rospy.loginfo('Side from %s', self.pose)
            self.advance(side_length, speed=speed)
            rospy.loginfo('Turn from %s', self.pose)
            self.turn(math.pi / 2, angular_speed=(speed / axis_length))
            self.sleep(1)
        rospy.loginfo('Done at %s', self.pose)
        self.sleep(1)

    def sleep(self, dt):
        rospy.sleep(dt)

    def update_odom(self, msg):
        position = msg.pose.pose.position
        angle = 2.0 * math.asin(msg.pose.pose.orientation.z)
        self.pose = (position.x, position.y, angle)

    def stop(self):
        self.pub.publish(Twist())

    def advance(self, length, speed):
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)
        self.sleep(length / speed)
        self.stop()

    def turn(self, angle, angular_speed):
        msg = Twist()
        msg.angular.z = angular_speed
        self.pub.publish(msg)
        self.sleep(angle / angular_speed)
        self.stop()


def main():
    FollowSquare()


if __name__ == '__main__':
    main()
