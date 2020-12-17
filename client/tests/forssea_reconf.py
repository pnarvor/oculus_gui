#! /usr/bin/python

from __future__ import print_function

import rospy

from dynamic_reconfigure.client import Client as ReconfClient

rospy.init_node('narval_monitor', anonymous=True, disable_signals=True)

config = 0
def config_callback(cfg):
    global config
    print("Got config")
    config = cfg

client = ReconfClient('ueye_cam_nodelet', timeout=20,
                      config_callback = config_callback)
d = client.get_parameter_descriptions()
