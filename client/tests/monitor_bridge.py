#! /usr/bin/python

from __future__ import print_function

import rospy

from narval_monitor import Session
from narval_monitor.http import HttpSession
from narval_monitor import OculusMonitor

from twisted.internet import reactor
from narval_monitor.reconfigure_bridge import ReconfigureBridgeFactory

rospy.init_node('narval_monitor', anonymous=True, disable_signals=True)

# session = Session('http://127.0.0.1:8000/payload_monitor')
session = HttpSession(reactor, '127.0.0.1', rootUrl='payload_monitor/')
sonarMonitor = OculusMonitor(pingTopic='/ping', http=session)

factory = ReconfigureBridgeFactory('oculus_sonar', 'ws://127.0.0.1:8000/ws/reconfigure_bridge/oculus_sonar/');

reactor.connectTCP("127.0.0.1", 8000, factory)
reactor.run()



