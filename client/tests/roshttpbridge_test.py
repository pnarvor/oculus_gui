#! /usr/bin/python

from __future__ import print_function

import rospy

import oculus_sonar.msg as oculus_msg
from narval_monitor.sonar_conversions import *

from narval_monitor.reconfigure_bridge import ReconfigureBridgeFactory

from twisted.internet        import reactor
from narval_monitor.http     import HttpSession
from narval_monitor.ros_http import RosHttpBridge

def ping_conversion(msg):
    return {'scalars' : from_OculusPing(msg),
            'vectors' : {'data' : ('data', msg.data)}}

class OculusMonitor(RosHttpBridge):

    def __init__(self, http):
        topics = [{'name' : '/ping', 'type' : oculus_msg.OculusPing,
                  'converter' : ping_conversion}]
        super(OculusMonitor, self).__init__(http, topics)
        
rospy.init_node('narval_monitor', anonymous=True, disable_signals=True)

session = HttpSession(reactor, '127.0.0.1', rootUrl='payload_monitor/')
session.connect()
sonarMonitor = OculusMonitor(session)

factory = ReconfigureBridgeFactory('oculus_sonar', 'ws://127.0.0.1:8000/ws/reconfigure_bridge/oculus_sonar/');

reactor.connectTCP("127.0.0.1", 8000, factory)
reactor.run()

sonarMonitor.stop()
