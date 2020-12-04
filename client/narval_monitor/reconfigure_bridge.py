
from __future__ import print_function

import sys
import ast
sys.path.append('../')
import json

from dynamic_reconfigure.client import Client as ReconfClient

from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory
from twisted.internet.protocol  import ReconnectingClientFactory

class ReconfigureBridgeProtocol(WebSocketClientProtocol):

    def __init__(self, targetNodeName, reconfClient, factory):
        super(WebSocketClientProtocol, self).__init__()
        self.targetNodeName = targetNodeName
        self.reconfClient   = reconfClient
        self.factory        = factory
        self.description    = None

    def onOpen(self):
        self.factory.add_protocol(self)
        if self.description is None:
            self.description = self.reconfClient.get_parameter_descriptions()
            for param in self.description:
                if len(param['edit_method']) > 0 and isinstance(param['edit_method'], str):
                    param['edit_method'] = ast.literal_eval(param['edit_method']);
        self.sendMessage(json.dumps({'type'    : 'description',
                                     'target'  : self.targetNodeName,
                                     'payload' : self.description}))
        print("Connection successful")

    def onMessage(self, msg, binary):
        data = json.loads(msg)
        if data['type'] == 'config_request':
            self.reconfClient.update_configuration(data['payload'])

    def onClose(self, wasClean, code, reason):
        self.factory.remove_protocol(self)
        if not wasClean:
            print("Bad close (code " + str(code) + "), reason :", reason)


class ReconfigureBridgeFactory(WebSocketClientFactory, ReconnectingClientFactory):

    protocol = ReconfigureBridgeProtocol
    
    def __init__(self, targetNodeName, url):
        # super(ReconnectingClientFactory, self).__init__() not working ?
        super(WebSocketClientFactory, self).__init__(url)
        self.targetNodeName = targetNodeName
        self.reconfClient = ReconfClient(self.targetNodeName, timeout=20,
                                         config_callback = self.config_callback)
        self.openedProtocols = {}
        
        # reconnection parameters
        self.maxDelay     = 3.0
        self.initialDelay = 0.25
        self.factor       = 1.5

    def buildProtocol(self, addr):
        return ReconfigureBridgeProtocol(self.targetNodeName, self.reconfClient, self)
    
    def add_protocol(self, protocol):
        self.openedProtocols[id(protocol)] = protocol

    def remove_protocol(self, protocol):
        del self.openedProtocols[id(protocol)]

    def config_callback(self, rosMsg):
        wsMsg = json.dumps({'type'    : 'config',
                            'target'  : self.targetNodeName,
                            'payload' : rosMsg})
        for ws in self.openedProtocols.values():
            ws.sendMessage(wsMsg)
    
    def clientConnectionFailed(self, connector, reason):
        print("Client connection failed. Retrying...")
        self.retry(connector)
    
    def clientConnectionLost(self, connector, reason):
        print("Client connection lost. Retrying...")
        self.retry(connector)
