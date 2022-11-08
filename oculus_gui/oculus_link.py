import uuid
import json
import threading
import oculus_python

from .cache import cache
from .data_serialization import serialize

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

        serialized = serialize(type(metadata).__name__, metadata, data)
        # print(serialized)

        msg = {'type' : 'empty', 'scalars' : 'None', 'vectors' : 'None'}
        if 'scalars' in serialized.keys():
            msg['scalars'] = json.dumps(serialized['scalars'])
            msg['type']    = 'form_data'
        if 'vectors' in serialized.keys():
            msg['type']    = 'cached_data'
            msg['vectors'] = {}
            for name, data in serialized['vectors'].items():
                dataUuid = cache.insert(data[1])
                msg['vectors'][name] = {
                    'data_uuid' : dataUuid,
                    'cache_request_uri' : '/oculus_gui/get_cached_data/'}

        encoded = json.dumps(msg, ensure_ascii=True)
        callbacks = []
        with self.lock:
            callbacks = [c for c in self.ping_callbacks.values()]
        for c in callbacks:
            c(encoded)



