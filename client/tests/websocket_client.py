from __future__ import print_function

import sys
from twisted.internet import reactor

from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory

class NarvalClientProtocol(WebSocketClientProtocol):

    def onOpen(self):
        print("Openened")

    def onMessage(self, msg, binary):
        print("Got message")
        print(msg)

    def onClose(self, wasClean, code, reason):
        print("Closing")

factory = WebSocketClientFactory('ws://127.0.0.1:8000/ws/subscribe/');
factory.protocol = NarvalClientProtocol
reactor.connectTCP("127.0.0.1", 8000, factory)
reactor.run()
