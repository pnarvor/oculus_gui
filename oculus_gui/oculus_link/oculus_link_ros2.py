import uuid
import json
import threading

import rclpy
from rclpy.node import Node
import oculus_interfaces.msg as oculus_msg

from .oculus_converters_ros2 import serializers
from .oculus_parameters import parameter_description
from .oculus_parameters import from_OculusSimpleFireMessage
from .oculus_parameters import update_OculusSimpleFireMessage

class OculusLink(Node):
    
    def __init__(self):
        rclpy.init()
        super().__init__('oculus_gui')

        self.pingCallbacks       = {}
        self.configCallbacks = {}
        self.lock = threading.Lock();

        self.pingSub = self.create_subscription(oculus_msg.OculusStampedPing,
                                                '/ping',
                                                self.ping_callback,
                                                10)
        # self.sonar.add_ping_callback(self.ping_callback)
        # self.sonar.add_config_callback(self.config_callback)
        # self.sonar.start()

    def get_parameter_description(self):
        # return parameter_description(self.sonar.current_config())
        return []

    def add_ping_callback(self, callback):
        with self.lock:
            callbackId = str(uuid.uuid4())
            self.pingCallbacks[callbackId] = callback
        return callbackId

    def remove_ping_callback(self, callbackId):
        with self.lock:
            del self.pingCallbacks[callbackId]

    def ping_callback(self, msg):

        callbacks = []
        with self.lock:
            callbacks = [c for c in self.pingCallbacks.values()]

        if len(callbacks) == 0:
            return

        serialized = serializers[type(msg).__name__][1](msg)

        for c in callbacks:
            c(serialized)

    def add_config_callback(self, callback):
        with self.lock:
            callbackId = str(uuid.uuid4())
            self.configCallbacks[callbackId] = callback
        return callbackId

    def remove_config_callback(self, callbackId):
        with self.lock:
            del self.configCallbacks[callbackId]

    def config_callback(self, lastConfig, newConfig):
        callbacks = []
        with self.lock:
            callbacks = [c for c in self.configCallbacks.values()]

        if len(callbacks) == 0:
            return
        config = from_OculusSimpleFireMessage(newConfig)
        for c in callbacks:
            c(config)

    def reconfigure(self, request):
        pass
        # config = self.sonar.current_config()
        # newConfig = update_OculusSimpleFireMessage(config, request)
        # self.sonar.send_config(newConfig)
        
