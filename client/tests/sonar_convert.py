#! /usr/bin/python

from __future__ import print_function

import sys
import rospy

import oculus_sonar.msg as oculus_msg

sys.path.append('../')
from narval_monitor import PayloadSession
from narval_monitor.sonar_conversions import *


ping = 0
def ping_callback(pingMsg):
    global ping
    # print(pingMsg)
    # print("got ping")
    print(oculus_from_ros(pingMsg))
    ping = pingMsg

rospy.init_node('sonar_monitor', anonymous=True)
rospy.Subscriber('/ping', oculus_msg.OculusPing, ping_callback)

rospy.spin()

