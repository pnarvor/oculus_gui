from __future__ import print_function

import rospy
import json

class RosHttpBridge(object):

    """
    RosHttpBridge

    This class forward ros messages via an http message to a http server using
    post requests.

    This was created more as a learning exercice than a real usable component
    (if you wish to transfer ros messages to a web browser, you should probably
    take a look at rosbridge instead).
    """
    def __init__(self, http, topics):
        """
        topics should contain a list of dict containing the name on the topic,
        the message type and a conversion callback to transform the ros
        messages in a data structure which can be posted through a post request.
        """
        self.http = http
        self.subscribers = {}
        for topic in topics:
            self.subscribers[topic['name']] = rospy.Subscriber(
                topic['name'], topic['type'],
                lambda msg: self.post_message(topic['name'], topic['converter'](msg)))
    
    def stop(self):
        for s in self.subscribers.values():
            s.unregister()
    
    def topic_to_uri(self, topicName):
        if topicName[0] == '/':
            return topicName[1:].replace('/', '_')
        else:
            return topicName.replace('/', '_')

    def post_message(self, topicName, msgData):
        uri = self.http.child_path('post_data/' + self.topic_to_uri(topicName))
        
        postData = {}
        if 'scalars' in msgData.keys():
            postData['data']  = {'scalars' : json.dumps(msgData['scalars'],
                                                        ensure_ascii=True)}
        if 'vectors' in msgData.keys():
            postData['files'] = msgData['vectors']

        d = self.http.post(uri, **postData)
        return d
