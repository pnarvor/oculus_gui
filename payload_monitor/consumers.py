
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
        pass

    def update(self, ping_data):
        # self.send(text_data=json.dumps({'message':'got ping'}))
        self.send(bytes_data=ping_data)
