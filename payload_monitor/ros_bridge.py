import json
from channels.generic.websocket import WebsocketConsumer

import rospy

from .ros_conversions import ros_converters

node = rospy.init_node("ros_bridge", disable_signals=True)

from .cache import cache
bridges = {}

class TopicBridge:

    def __init__(self, topicName, topicTypeName):
        self.topicName      = topicName
        self.topicTypeName  = topicTypeName
        self.topicType      = ros_converters[self.topicTypeName][0]
        self.numReceivedMsg = 0
        self.subscribers    = {}
        self.rosSubscriber  = None
        self.converter      = None

    def msg_callback(self, rosMsg):
        if self.converter is None:
            return
        rosData = self.converter(rosMsg)
        msg = {'topic' : self.topicName, 'type' : 'empty', 
               'scalars' : 'None', 'vectors' : 'None'}
        if 'scalars' in rosData.keys():
            msg['scalars'] = json.dumps(rosData['scalars'])
            msg['type']    = 'form_data'
        if 'vectors' in rosData.keys() and len(rosData['vectors']) > 0:
            msg['type']    = 'cached_data'
            msg['vectors'] = {}
            for name, data in rosData['vectors'].items():
                dataUuid = cache.insert(data[1])
                msg['vectors'][name] = {
                    'data_uuid'         : dataUuid,
                    'cache_request_uri' : '/payload_monitor/get_cached_data/'}
        # print(msg)
        for sub in self.subscribers.values():
            sub.update(text_data=json.dumps(msg, ensure_ascii=True))

    def add_subscriber(self, subscriber):
    
        if subscriber.topicName != self.topicName:
            return False
        if subscriber.topicType != self.topicType:
            return False

        self.subscribers[id(subscriber)] = subscriber

        if self.rosSubscriber is None:
            print("Subscribing to ros topic :", self.topicName, flush=True)
            self.rosSubscriber = rospy.Subscriber(self.topicName, self.topicType,
                                                  self.msg_callback)
            self.converter = ros_converters[self.topicTypeName][1]
        return True

    def remove_subscriber(self, subscriber):
        if id(subscriber) in self.subscribers.keys():
            del self.subscribers[id(subscriber)]
        if len(self.subscribers) == 0:
            print("Unsubscribing from topic :", self.topicName)
            self.rosSubscriber.unregister()
            self.rosSubscriber = None


class Subscriber(WebsocketConsumer):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def connect(self):

        try:
            arguments = self.scope["url_route"]["kwargs"]
            self.topicName = arguments["topicName"].replace('-', '/');
            
            # Converting type string to ros message type
            self.topicTypeName = arguments["topicType"].replace('-', '/');
            if self.topicTypeName not in ros_converters.keys():
                print("No converter for type " + self.topicTypeName + 
                      ". Ignoring subscription to topic " + self.topicName + ".")
                return
            self.topicType = ros_converters[self.topicTypeName][0]
        
            if self.topicName not in bridges.keys():
                bridges[self.topicName] = TopicBridge(self.topicName,
                                                      self.topicTypeName)
            if not bridges[self.topicName].add_subscriber(self):
                return
            print("Subscribing to :", self.topicName, flush=True)
            self.accept()
        except Exception as e:
            print(e)

    def disconnect(self, closeCode):
        if self.topicName in bridges.keys():
            bridges[self.topicName].remove_subscriber(self)
    
    def receive(self, data):
        print("Got data from ws client :", data)

    def update(self, text_data=None, bytes_data=None):
        self.send(text_data=text_data, bytes_data=bytes_data)









