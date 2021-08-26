#!/usr/bin/env python

import rospy

from thymio_driver.thymio_driver_node import ThymioDriver


def main() -> None:
    rospy.init_node('driver')
    ThymioDriver(namespace='', standalone=True)
    rospy.spin()


if __name__ == '__main__':
    main()
