#! /usr/bin/python

from __future__ import print_function

import rospy

import oculus_sonar.msg as oculus_msg
from narval_monitor.sonar_conversions import *
from sensor_msgs.msg import Image as ImageMsg

from narval_monitor.reconfigure_bridge import ReconfigureBridgeFactory

from twisted.internet        import reactor
# from narval_monitor.http     import HttpSession
from narval_monitor import Session
from narval_monitor.ros_http import RosHttpBridge

def ping_conversion(msg):
    return {'scalars' : from_OculusPing(msg),
            'vectors' : {'data' : ('data', msg.data)}}
class OculusMonitor(RosHttpBridge):

    def __init__(self, http):
        topics = [{'name' : '/ping', 'type' : oculus_msg.OculusPing,
                  'converter' : ping_conversion}]
        super(OculusMonitor, self).__init__(http, topics)

def image_conversion(msg):
    return {'scalars' : {'width'       : msg.width,
                         'height'      : msg.height,
                         'encoding'    : msg.encoding,
                         'isbigendian' : msg.is_bigendian,
                         'step'        : msg.step},
            'vectors' : {'data' : ('data', msg.data)}}
class CameraMonitor(RosHttpBridge):

    def __init__(self, http):
        topics = [{'name' : '/camera/image_raw', 'type' : ImageMsg,
                   'converter' : image_conversion}]
        super(CameraMonitor, self).__init__(http, topics)
        
rospy.init_node('narval_monitor', anonymous=True, disable_signals=True)

# session = HttpSession(reactor, '127.0.0.1', rootUrl='payload_monitor/')
session = Session('http://127.0.0.1:8000/payload_monitor/')
# session.connect()

sonarMonitor  = OculusMonitor(session)
cameraMonitor = CameraMonitor(session)

sonarReconf    = ReconfigureBridgeFactory('oculus_sonar',
    'ws://127.0.0.1:8000/ws/reconfigure_bridge/oculus_sonar/');
reactor.connectTCP("127.0.0.1", 8000, sonarReconf)

recorderReconf = ReconfigureBridgeFactory('narval_recorder',
    'ws://127.0.0.1:8000/ws/reconfigure_bridge/narval_recorder/');
reactor.connectTCP("127.0.0.1", 8000, recorderReconf)

cameraReconf = ReconfigureBridgeFactory('ueye_cam_nodelet',
    'ws://127.0.0.1:8000/ws/reconfigure_bridge/ueye_cam_nodelet/');
reactor.connectTCP("127.0.0.1", 8000, cameraReconf)

reactor.run()

sonarMonitor.stop()
cameraMonitor.stop()


