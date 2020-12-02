#! /usr/bin/python

from __future__ import print_function

import sys
sys.path.append('../')
import rospy
import dynamic_reconfigure.client

from narval_monitor import ReconfigureParser

configMsg = 0;
parsedConfig = 0;
def config_description_callback(msg):
    global configMsg
    global parsedConfig
    configMsg = msg
    print("Got config message")
    parsedConfig = ReconfigureParser().parse_config_desc(msg)


rospy.init_node("sonar_ctrl1", anonymous=True)

s = rospy.Subscriber('/oculus_sonar/parameter_descriptions',
                     dynamic_reconfigure.msg.ConfigDescription,
                     config_description_callback)

rospy.spin()


