# python 2-3 compatibility imports
from __future__ import print_function

import rospy

import oculus_sonar.msg as oculus_msg

from .sonar_conversions import *

class OculusMonitor:

    def __init__(self, pingTopic=None, statusTopic=None, http=None):
        self.pingTopic   = pingTopic
        self.statusTopic = statusTopic
        self.http        = http

        if pingTopic is not None:
            self.pingPostUrl = self.generate_topic_url(pingTopic)
            rospy.Subscriber(pingTopic, oculus_msg.OculusPing,
                             self.ping_callback)
        if statusTopic is not None:
            rospy.Subscriber(statusTopic, oculus_msg.Oculusstatus,
                             self.status_callback)

    def generate_topic_url(self, topicName):
        if topicName[0] == '/':
            res = topicName[1:]
        else:
            res = topicName
        return '/post_data/' + res.replace('/', '-')


    def ping_callback(self, msg):
        if self.http is None:
            return
        response = self.http.post_message(self.pingPostUrl,
                                          from_OculusPing(msg), msg.data)
        # response = self.http.post_message('generic_post',
        #                                   from_OculusPing(msg), msg.data)
        # if response is not None:
        #     print(response)
        #     print(response.headers)
    
    def status_callback(self, msg):
        pass

            
