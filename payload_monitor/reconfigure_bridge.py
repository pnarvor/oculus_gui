import json

from channels.generic.websocket import WebsocketConsumer

bridges = {}

class ReconfigureBridge(WebsocketConsumer):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.clients = {}
        self.currentConfig = None
        self.description   = None

    def connect(self):
        self.nodeName = self.scope['url_route']['kwargs']['nodeName']
        if self.nodeName in bridges.keys():
            self.close(code=3254)
            return
        bridges[self.nodeName] = self
        self.accept()

    def disconnect(self, closeCode):
        del bridges[self.nodeName]
    
    def receive(self, text_data=None, bytes_data=None, close=False):
        data = json.loads(text_data)
        if data['type'] == 'description':
            self.description = data['payload']
        elif data['type'] == 'config':
            self.currentConfig = data['payload']
        for client in self.clients.values():
            client.update(text_data)

    def add_client(self, client):
        self.clients[id(client)] = client
        if self.description is not None:
            if self.currentConfig is not None:
                for param in self.description:
                    param['current_value'] = self.currentConfig[param['name']]
            client.update(json.dumps({'type'    : 'description', 
                                      'target'  : self.nodeName,
                                      'payload' : self.description}))
    
    def remove_client(self, client):
        del self.clients[id(client)]


class ReconfigureClient(WebsocketConsumer):
    
    def connect(self):
        self.nodeName = self.scope['url_route']['kwargs']['nodeName']
        if not self.nodeName in bridges.keys():
            self.close(code=3255)
            return
        self.accept()
        bridges[self.nodeName].add_client(self)
        
    def disconnect(self, closeCode):
        bridges[self.nodeName].remove_client(self)

    def update(self, config):
        self.send(text_data=config)

    def receive(self, text_data=None, bytes_data=None, close=False):
        bridges[self.nodeName].send(
            text_data=text_data, bytes_data=bytes_data, close=close)
