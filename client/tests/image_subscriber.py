#! /usr/bin/python

from __future__ import print_function

import rospy
import numpy as np

from sensor_msgs.msg import Image as ImageMsg

def image_conversion(msg):
    return {'scalars' : {'width'       : msg.width,
                         'height'      : msg.height,
                         'encoding'    : msg.encoding,
                         'isbigendian' : msg.is_bigendian,
                         'step'        : msg.step},
            'vectors' : {'data' : ('data', msg.data)}}

msg = None
def image_callback(message):
    global msg
    if msg is None:
        print("Got image")
        msg = message
        
rospy.init_node('narval_monitor', anonymous=True, disable_signals=True)

subscriber = rospy.Subscriber('/camera/image_raw', ImageMsg, image_callback)

