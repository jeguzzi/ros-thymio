#!/usr/bin/env python

import rospy

from thymio_driver.manager import Manager
from thymio_driver.thymio_driver_node import ThymioDriver


class ThymioManager(Manager):

    _drivers = {'thymio-II': ThymioDriver}


def main() -> None:
    rospy.init_node('manager')
    ThymioManager()
    rospy.spin()


if __name__ == '__main__':
    main()
