

import json

from channels.generic.websocket import WebsocketConsumer

from . import oculus_link

sonarLink = oculus_link.OculusLink()
class OculusConsumer(WebsocketConsumer):

    def connect(self):
        global sonarLink
        self.callbackId = sonarLink.add_ping_callback(self.send_data)
        self.accept()

    def disconnect(self, closeCode):
        global sonarLink
        sonarLink.remove_ping_callback(self.callbackId)
        print("OculusConsumer disconnected (code " + str(closeCode) + ")")

    def send_data(self, text_data):
        self.send(text_data=text_data)
        # print("Got ping data :", text_data)
        # print("Got ping data :")


