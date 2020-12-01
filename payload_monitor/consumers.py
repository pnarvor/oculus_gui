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
        print("url argument", self.scope["url_route"]["kwargs"]["argument"])
        self.accept()
        register[id(self)] = self

    def disconnect(self, closeCode):
        del register[id(self)]
    
    def receive(self, data):
        print("Got data from ws client :", data)

    def update(self, text_data=None, bytes_data=None):
        self.send(text_data=text_data, bytes_data=bytes_data)


class WebSocketTest(WebsocketConsumer):

    def __init__(self, *args, **kwargs):
        print("Building websocket :", *args, **kwargs)
        self.messageCount = 0
        super().__init__(*args, **kwargs)

    def connect(self):
        self.accept()
        # # this is not sending user data (I think...)
        # super(WebsocketConsumer, self).send({"type": "websocket.accept", "subprotocol": None,
        #                                      "user_data" : "have some !"})

        self.send(text_data="text")
        super(WebsocketConsumer, self).send({"type": "websocket.send", "text": "here !"})
        super(WebsocketConsumer, self).send({"type": "websocket.send", "bytes": b"here !"})

        # # not working (daphne rejects)
        # super(WebsocketConsumer, self).send({"type": "websocket.send",
        #                                      "text": "here !",
        #                                      "bytes": b"here !"})


    def disconnect(self, closeCode):
        print("WebSocket disconnected. Code :", closeCode)
    
    def receive(self, text_data=None, bytes_data=None, close=False):
        self.messageCount += 1
        print("WebSocket : Got data")
        print(type(text_data), text_data)
        print(type(bytes_data), bytes_data)
        print(type(close), close)
        print("message count :", self.messageCount)
        
        if self.messageCount < 20:
        # if self.messageCount < 50:
            self.send("Me too !")
        else:
            # self.close(3256)
            self.close()

    # def update(self, text_data=None, bytes_data=None):
    #     self.send(text_data=text_data, bytes_data=bytes_data)
