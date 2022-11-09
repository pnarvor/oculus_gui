import uuid
import json
import threading
import oculus_python

from .oculus_converters import serializers
from .oculus_parameters import parameter_description
from .oculus_parameters import from_OculusSimpleFireMessage
from .oculus_parameters import update_OculusSimpleFireMessage

class OculusLink:
    
    def __init__(self):

        self.pingCallbacks       = {}
        self.configCallbacks = {}
        self.lock = threading.Lock();

        self.sonar = oculus_python.OculusSonar()
        self.sonar.add_ping_callback(self.ping_callback)
        self.sonar.add_config_callback(self.config_callback)
        self.sonar.start()

    def get_parameter_description(self):
        return parameter_description(self.sonar.current_config())

    def add_ping_callback(self, callback):
        with self.lock:
            callbackId = str(uuid.uuid4())
            self.pingCallbacks[callbackId] = callback
        return callbackId

    def remove_ping_callback(self, callbackId):
        with self.lock:
            del self.pingCallbacks[callbackId]

    def ping_callback(self, metadata, data):

        callbacks = []
        with self.lock:
            callbacks = [c for c in self.pingCallbacks.values()]

        if len(callbacks) == 0:
            return

        serialized = serializers[type(metadata).__name__][1](metadata, data)

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
        config = self.sonar.current_config()
        newConfig = update_OculusSimpleFireMessage(config, request)
        self.sonar.send_config(newConfig)
        
