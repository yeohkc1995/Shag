#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
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

### KC: import GPS NavSatFix Message and pyprog(for GPS coordinates calulation) ###
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
#import pyproj
from Haversine import Haversine_XY
from std_msgs.msg import Float64
import time

class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PoseStamped()
        self.radius = 1

        ### KC: Subscribe to topic to listen for zoon's GPS msg and GCS command ###
        rospy.Subscriber("zoon_coordinates", NavSatFix, self.callback)
        rospy.Subscriber("mission_control", String, self.callback2)
        rospy.Subscriber("zoon_heading", Float64, self.callback_heading)
        
        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        ### KC: zoon_coord
        self.zoon_lat = 0
        self.zoon_long = 0
        self.zoon_alt = 0

        ### KC: Mission control variable
        self.end_mission = 0
        self.safe_dist = 0
        self.hdg_offset = 0

        ### KC: Leader Magnetic Heading ###
        self.leader_hdg = 0

        ### KC: Heading time ###
        self.hdg_time = 0

    ### KC: function that activates when message from zoon is recevied ###
    def callback(self, data):
        rospy.loginfo("\nLatitude: %s\nLongitude: %s\nAttitude: %s", data.latitude, data.longitude, data.altitude)
        self.zoon_lat = data.latitude
        self.zoon_long = data.longitude
        self.zoon_alt = data.altitude
    
    ### KC: subscriber function to take input from mission control GUI
    def callback2(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == "N0":
            self.end_mission = 1
        elif data.data[0].upper() == 'E':
            if len(data.data) == 1:
                self.safe_dist = 10
            elif 10 <= int(data.data[1:]) <= 20:
                self.safe_dist = int(data.data[1:])
        elif data.data[0].upper() == 'N':
            if int(data.data[1:]) == 1:
                self.hdg_offset = 180
            elif int(data.data[1:]) == 2:
                self.hdg_offset = 0
            elif int(data.data[1:]) == 3:
                 self.hdg_offset = 270
            elif int(data.data[1:]) == 4:
                 self.hdg_offset = 90
        else:
            pass
    
    ### KC: subscriber funtion to pull compass heading data every 2 seconds
    def callback_heading(self, data):
        if (time.time() - self.hdg_time) < 2:
            return
            
        self.hdg_time = time.time()
        rospy.loginfo("\nLeader Heading: %s\n", data.data)
        self.leader_hdg = data.data

        


    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        #self.assertTrue(reached, (
        #    "Refreshing destination coordinates.... | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
        #    format(self.local_position.pose.position.x,
        #           self.local_position.pose.position.y,
        #           self.local_position.pose.position.z, timeout)))

    def Harversine(self, lat1, long1, lat2, long2):
        pi = math.pi
        lat1 *=pi/180
        lat2 *=pi/180
        long1*=pi/180
        long2*=pi/180

        dlong = (long2 - long1)
        dlat  = (lat2 - lat1)

        #Haversine formula:
        R = 6371
        a = math.sin(dlat/2)*math.sin(dlat/2) + math.cos(lat1)*math.cos(lat2)*math.sin(dlong/2)*math.sin(dlong/2)
        c = 2 * math.atan2( math.sqrt(a), math.sqrt(1-a) )
        d = R * c

        return c, d

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        ### KC: Grab the initial coordinates of the drone at take-off point. Also let the 1st zoon coord be the starting point
        self.initial_coord = rospy.wait_for_message("/mavros/global_position/global", NavSatFix)
        rospy.loginfo("\n\n\n\nInitial Lat: %s\nInitial Long: %s\nInitial Alt: %s\n\n\n", self.initial_coord.latitude, self.initial_coord.longitude, self.initial_coord.altitude)
        self.zoon_lat = self.initial_coord.latitude
        self.zoon_long = self.initial_coord.longitude
        self.zoon_alt = self.initial_coord.altitude

        rospy.loginfo("run mission")

        ### KC: First iteration, 30 seconds timelimit
        x, y, d = Haversine_XY(self.initial_coord.latitude, self.initial_coord.longitude, self.zoon_lat, self.zoon_long)
        theta = (self.leader_hdg + self.hdg_offset) % 360
        x += self.safe_dist * math.sin(math.radians(theta))
        y += self.safe_dist * math.cos(math.radians(theta))
        self.reach_position(x, y, 20, 30)

        ### KC: while the end_mission is not activated, keep going to the zoon coordinate###
        while self.end_mission != 1:
            x, y, d = Haversine_XY(self.initial_coord.latitude, self.initial_coord.longitude, self.zoon_lat, self.zoon_long)
            theta = (self.leader_hdg + self.hdg_offset) % 360
            x += self.safe_dist * math.sin(math.radians(theta))
            y += self.safe_dist * math.cos(math.radians(theta))
            self.reach_position(x, y, 20, 2)
        

        self.reach_position(0, 0, 20, 30)
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
