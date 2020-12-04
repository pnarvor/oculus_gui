#! /usr/bin/python

from __future__ import print_function

import sys
import rospy

import oculus_sonar.msg as oculus_msg

sys.path.append('../')
from narval_monitor import Session
from narval_monitor.sonar_conversions import *


session = Session('http://127.0.0.1:8000/payload_monitor')

def ping_callback(msg):
    global session
    metadata = from_OculusPing(msg)
    response = session.post_message('/post_data', metadata, msg.data)
    if response is not None:
        print(response)
        print(response.headers)

rospy.init_node('sonar_monitor', anonymous=True, disable_signals=True)
rospy.Subscriber('/ping', oculus_msg.OculusPing, ping_callback)

rospy.spin()

