
from __future__ import print_function

import sys
import ast
sys.path.append('../')
import json

from dynamic_reconfigure.client import Client as ReconfClient

from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory

class ReconfigureBridgeProtocol(WebSocketClientProtocol):

    def __init__(self, targetNodeName):
        super(WebSocketClientProtocol, self).__init__()
        self.targetNodeName = targetNodeName

        self.reconfClient = ReconfClient(self.targetNodeName, timeout=20,
                                         config_callback = self.config_callback)

    def config_callback(self, msg):
        print("Got config")
        self.sendMessage(json.dumps({'type'    : 'config',
                                     'payload' : msg}))

    def onOpen(self):
        self.description = self.reconfClient.get_parameter_descriptions()
        for param in self.description:
            if len(param['edit_method']) > 0:
                param['edit_method'] = ast.literal_eval(param['edit_method']);
        self.sendMessage(json.dumps({'type'    : 'description',
                                     'payload' : self.description}))
        print("Connection successful")

    def onMessage(self, msg, binary):
        print("Got message")
        print(msg)
        data = json.loads(msg)
        if data['type'] == 'config_request':
            self.reconfClient.update_configuration(data['payload'])

    def onClose(self, wasClean, code, reason):
        print("Closing", wasClean, code, reason)


class ReconfigureBridgeFactory(WebSocketClientFactory):
    
    def __init__(self, targetNodeName, url):
        # self.protocol = ReconfigureBridgeProtocol # ?
        super(WebSocketClientFactory, self).__init__(url)
        self.targetNodeName = targetNodeName

    def buildProtocol(self, addr):
        res = ReconfigureBridgeProtocol(self.targetNodeName)
        res.factory = self # ?
        return res


