

import json

from channels.generic.websocket import WebsocketConsumer


class OculusConsumer(WebsocketConsumer):

    def connect(self):
        print("New client connection")
        self.accept()

    def disconnect(self):
        pass

    def receive(self, text_data):
        text_data_json = json.loads(text_data)
        message = text_data_json['message']

        self.send(text_data=json.dumps({'message': message}))


