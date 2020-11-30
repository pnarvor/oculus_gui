import json
from channels.generic.websocket import WebsocketConsumer

register = {}

class PingConsumer(WebsocketConsumer):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def connect(self):
        self.accept()
        register[id(self)] = self

    def disconnect(self):
        del register[id(self)]

    def update(self, text_data=None, bytes_data=None):
        print("there", flush=True)
        print("base_send", self.base_send, flush=True)
        # self.send(text_data=json.dumps({'message':'got ping'}))
        self.send(text_data=text_data, bytes_data=bytes_data)


class DataUpdater(WebsocketConsumer):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def connect(self):
        self.accept()
        register[id(self)] = self

    def disconnect(self, closeCode):
        del register[id(self)]
    
    def receive(self, data):
        print("Got data from ws client :", data)

    def update(self, text_data=None, bytes_data=None):
        self.send(text_data=text_data, bytes_data=bytes_data)
