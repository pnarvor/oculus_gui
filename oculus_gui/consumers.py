import json

from channels.generic.websocket import WebsocketConsumer

from .cache import cache

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

    def send_data(self, serialized):
        msg = {'type' : 'empty', 'scalars' : 'None', 'vectors' : 'None'}
        if 'scalars' in serialized.keys():
            msg['scalars'] = json.dumps(serialized['scalars'])
            msg['type']    = 'form_data'
        if 'vectors' in serialized.keys():
            msg['type']    = 'cached_data'
            msg['vectors'] = {}
            for name, data in serialized['vectors'].items():
                dataUuid = cache.insert(data[1])
                msg['vectors'][name] = {
                    'data_uuid' : dataUuid,
                    'cache_request_uri' : '/oculus_gui/get_cached_data/'}
        self.send(text_data=json.dumps(msg, ensure_ascii=True))

        # self.send(text_data=text_data)

class ReconfigureConsumer(WebsocketConsumer):

    def connect(self):
        global sonarLink
        self.callbackId = sonarLink.add_ping_callback(self.send_description)
        self.accept()

    def disconnect(self, closeCode):
        global sonarLink
        sonarLink.remove_ping_callback(self.callbackId)
        print("ReconfigureConsumer disconnected (code " + str(closeCode) + ")")

    def send_description(self, data):
        if data['scalars']['type'] != 'OculusSimplePingResult':
            return
        # print(data['scalars'])
        global sonarLink
        sonarLink.remove_ping_callback(self.callbackId)
        self.callbackId = sonarLink.add_config_callback(self.send_current_config)

        description = sonarLink.get_parameter_description()
        self.send(json.dumps({'type'    : 'description',
                              'payload' :  description}))

    def send_current_config(self, config):
        self.send(json.dumps({'type'    : 'config',
                              'payload' : config}))

    def receive(self, text_data):
        data = json.loads(text_data)
        if data['type'] != 'config_request':
            return
        global sonarLink
        sonarLink.reconfigure(data['payload'])        


class RecorderConsumer(WebsocketConsumer):

    def connect(self):
        global sonarLink
        self.callbackId = sonarLink.add_status_callback(self.send_description)
        self.accept()

    def disconnect(self, closeCode):
        global sonarLink
        sonarLink.remove_ping_callback(self.callbackId)
        print("ReconfigureConsumer disconnected (code " + str(closeCode) + ")")

    def send_description(self, msg):
        global sonarLink
        sonarLink.remove_status_callback(self.callbackId)
        self.callbackId = sonarLink.add_status_callback(self.send_current_config)

        description = [{'name'          : 'recording',
                        'description'   : 'True if sonar is recording',
                        'type'          : 'bool',
                        'default'       : False,
                        'edit_method'   : {'type' : 'bool'},
                        'current_value' : sonarLink.is_recording()}]

        self.send(json.dumps({'type'    : 'description',
                              'payload' :  description}))

    def send_current_config(self, msg):
        global sonarLink
        config = {'recording' : sonarLink.is_recording()}
        self.send(json.dumps({'type'    : 'config',
                              'payload' : config}))

    def receive(self, text_data):
        data = json.loads(text_data)
        if data['type'] != 'config_request':
            return
        if 'recording' not in data['payload'].keys():
            return
        global sonarLink
        import os
        if data['payload']['recording']:
            filename = os.path.join(os.environ['HOME'], 'output.oculus')
            sonarLink.sonar.recorder_start(filename, True)
        else:
            sonarLink.sonar.recorder_stop()

