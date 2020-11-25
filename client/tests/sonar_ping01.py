#! /usr/bin/python

import sys
import rospy

import oculus_sonar.msg as oculus_msg

sys.path.append('../')
from narval_monitor import Session
from narval_monitor.sonar_conversions import *


session = Session('http://127.0.0.1:8000/payload_monitor')

# ping = 0
# def ping_callback(pingMsg):
#     global ping
#     print(pingMsg)
#     ping = pingMsg

def ping_callback(msg):
    metadata = from_OculusPing(msg)
    # print(metadata)
    session.post_message('/generic_update', metadata, msg.data)

rospy.init_node('sonar_monitor', anonymous=True)
rospy.Subscriber('/ping', oculus_msg.OculusPing, ping_callback)
# rospy.Subscriber('/ping', oculus_msg.OculusPing, session.post_sonar_ping)

rospy.spin()

