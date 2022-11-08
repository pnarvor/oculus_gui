

import json

from channels.generic.websocket import WebsocketConsumer

from . import oculus_link

sonarLink = oculus_link.OculusLink()

class OculusConsumer(WebsocketConsumer):

    def connect(self):
        global sonarLink
        self.callbackId = sonarLink.add_ping_callback(self.handle_ping)
        self.accept()

        print("New client connection")

    def disconnect(self, closeCode):
        global sonarLink
        sonarLink.remove_ping_callback(self.callbackId)
        print("OculusConsumer disconnected (code " + str(closeCode) + ")")

    def handle_ping(self, metadata, data):
        self.send(text_data=json.dumps({'message': 'test'}))
        # print("Got ping data :", metadata)
        print("Got ping data :")

    # def receive(self, text_data):
    #     text_data_json = json.loads(text_data)
    #     message = text_data_json['message']

    #     self.send(text_data=json.dumps({'message': message}))


