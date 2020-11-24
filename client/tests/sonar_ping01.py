#! /usr/bin/python

import sys
import rospy

sys.path.append('../')
from narval_monitor import PayloadSession

import oculus_sonar.msg as oculus_msg

session = PayloadSession('http://127.0.0.1:8000/payload_monitor')

ping = 0
def ping_callback(pingMsg):
    global ping
    print(pingMsg)
    ping = pingMsg

rospy.init_node('sonar_monitor', anonymous=True)
# rospy.Subscriber('/ping_ping_update', oculus_msg.OculusPing, ping_callback)
rospy.Subscriber('/ping', oculus_msg.OculusPing, session.post_sonar_ping)

rospy.spin()

