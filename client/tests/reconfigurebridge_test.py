from __future__ import print_function

import sys
sys.path.append('../')

import rospy
rospy.init_node("sonar_ctrl", anonymous=True, disable_signals=True)

from twisted.internet import reactor
from narval_monitor.reconfigure_bridge import ReconfigureBridgeFactory


factory = ReconfigureBridgeFactory('oculus_sonar', 'ws://127.0.0.1:8000/ws/reconfigure_bridge/oculus_sonar/');
reactor.connectTCP("127.0.0.1", 8000, factory)
reactor.run()
