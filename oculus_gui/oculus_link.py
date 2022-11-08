import uuid
import threading
import oculus_python

class OculusLink:
    
    def __init__(self):

        self.ping_callbacks = {}
        self.lock = threading.Lock();

        self.sonar = oculus_python.OculusSonar()
        self.sonar.add_ping_callback(self.ping_callback);
        self.sonar.start()

    def add_ping_callback(self, callback):
        with self.lock:
            callbackId = str(uuid.uuid4())
            self.ping_callbacks[callbackId] = callback
        return callbackId

    def remove_ping_callback(self, callbackId):
        with self.lock:
            del self.ping_callbacks[callbackId]

    def ping_callback(self, metadata, data):
        callbacks = []
        with self.lock:
            callbacks = [c for c in self.ping_callbacks.values()]
        for c in callbacks:
            c(metadata, data)


        
