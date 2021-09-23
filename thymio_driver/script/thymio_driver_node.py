#!/usr/bin/env python
# type: ignore
import rospy

from thymio_driver.thymio_driver_node import ThymioDriver


def main():
    rospy.init_node('driver')
    ThymioDriver()
    rospy.spin()


if __name__ == '__main__':
    main()
