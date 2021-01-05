from __future__ import print_function

import rospy
from sensor_msgs.msg import Image as ImageMsg

rospy.init_node('narval_monitor', anonymous=True, disable_signals=True)

msg = None
def callback(m):
    global msg
    if msg is None:
        msg = m

subscriber = rospy.Subscriber('/camera/image_raw', ImageMsg, callback)
