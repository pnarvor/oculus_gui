
from __future__ import print_function

import sys
sys.path.append('../')
from dynamic_reconfigure.client import Client as ReconfClient

class ReconfigureClient:

    def __init__(self, targetNodeName, djangleSession=None):
        self.targetNodeName = targetNodeName
        self.djangleSession = djangleSession

        self.reconfClient = ReconfClient(self.targetNodeName, timeout=20,
                                         config_callback = self.config_callback)

    def config_callback(self, config):
        print("Got config")
        print(config)


