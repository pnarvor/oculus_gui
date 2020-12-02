#! /usr/bin/python

from __future__ import print_function

import sys
import rospy

import oculus_sonar.msg as oculus_msg

sys.path.append('../')
from narval_monitor import Session
from narval_monitor.sonar_conversions import *

class RosAdapter:

    def __init__(self, 

session = Session('http://127.0.0.1:8000/payload_monitor')

def ping_callback(msg):

    metadata = from_OculusPing(msg)
    response = session.post_message('/post_data', metadata, msg.data)
    print(response)
    print(response.headers)

rospy.init_node('sonar_monitor', anonymous=True)
rospy.Subscriber('/ping', oculus_msg.OculusPing, ping_callback)

rospy.spin()


