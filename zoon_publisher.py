#!/usr/bin/env python
# license removed for brevity
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
import time

from sensor_msgs.msg import NavSatFix
from pprint import pprint
from std_msgs.msg import Float64


def talker():
    pub = rospy.Publisher('zoon_coordinates', NavSatFix, queue_size=10)
    pub2 = rospy.Publisher('zoon_heading', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    GPS_coord = NavSatFix()
    #pprint(vars(GPS_coord))
    GPS_coord.altitude = 50
    GPS_coord.latitude = 47.399301
    GPS_coord.longitude = 8.546684
    Heading = 0

    while not rospy.is_shutdown():
        rospy.loginfo(GPS_coord)
        rospy.loginfo(Heading)
        GPS_coord.latitude += 0.00001
        GPS_coord.longitude += 0.00001
        Heading = (Heading + 90)%360
        raw_input("Next coord...")
        pub.publish(GPS_coord)
        pub2.publish(Heading)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
