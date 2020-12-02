from __future__ import print_function

import sys
from twisted.internet import reactor

from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory

class NarvalClientProtocol(WebSocketClientProtocol):

    def __init__(self, param):
        super(WebSocketClientProtocol, self).__init__()
        print("Build !", param)

    def onOpen(self):
        print("Openened")

    def onMessage(self, msg, binary):
        print("Got message")
        print(msg)

    def onClose(self, wasClean, code, reason):
        print("Closing")

class NarvalClientFactory(WebSocketClientFactory):
    def __init__(self, param, *args, **kwargs):
        print(*args)
        print(**kwargs)
        super(WebSocketClientFactory, self).__init__(*args, **kwargs)
        self.param = param

    def buildProtocol(self, addr):
        print(addr)
        res = NarvalClientProtocol(self.param)
        res.factory = self
        return res
        


# factory = WebSocketClientFactory('hello', 'ws://127.0.0.1:8000/ws/subscribe/');
factory = NarvalClientFactory('hello', 'ws://127.0.0.1:8000/ws/subscribe/');
factory.protocol = NarvalClientProtocol
reactor.connectTCP("127.0.0.1", 8000, factory)
reactor.run()
