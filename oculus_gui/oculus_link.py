import uuid
import json
import threading
import oculus_python

from .data_serialization import serialize

from .oculus_parameters import parameter_description
from .oculus_parameters import from_OculusSimpleFireMessage
from .oculus_parameters import update_OculusSimpleFireMessage

class OculusLink:
    
    def __init__(self):

        self.pingCallbacks       = {}
        self.parametersCallbacks = {}
        self.lock = threading.Lock();

        self.sonar = oculus_python.OculusSonar()
        self.sonar.add_ping_callback(self.ping_callback)
        self.sonar.add_ping_callback(self.parameters_callback)
        self.sonar.start()

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

        serialized = serialize(type(metadata).__name__, metadata, data)

        for c in callbacks:
            c(serialized)

    def add_parameters_callback(self, callback):
        with self.lock:
            callbackId = str(uuid.uuid4())
            self.parametersCallbacks[callbackId] = callback
        return callbackId

    def remove_parameters_callback(self, callbackId):
        with self.lock:
            del self.parametersCallbacks[callbackId]

    def parameters_callback(self, metadata, data):
        callbacks = []
        with self.lock:
            callbacks = [c for c in self.parametersCallbacks.values()]

        if len(callbacks) == 0:
            return
        config = from_OculusSimpleFireMessage(self.sonar.current_config())
        for c in callbacks:
            c(config)
        

    def reconfigure(self, request):
        config = self.sonar.current_config()
        newConfig = update_OculusSimpleFireMessage(config, request)
        self.sonar.send_config(newConfig)
        
