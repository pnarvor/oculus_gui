#! /usr/bin/python

from __future__ import print_function

import time
import threading
from functools import partial
from uuid import uuid4

import rospy

from dynamic_reconfigure.client import Client as ReconfClient
from dynamic_reconfigure import find_reconfigure_services

class ReconfigureMuxer:

    @staticmethod
    def format_node_name(name):
        if name[0] != '/':
            return '/' + name
        else:
            return name
    
    def __init__(self, managedNodes=[]):
        # if empty, all nodes are managed
        self.managedNodes = [ReconfigureMuxer.format_node_name(n)
                             for n in managedNodes] 
        self.clients         = {}
        self.configCallbacks = {}
        self.callbackMap     = {}
        self.lock            = threading.Lock()

    def __str__(self):
        res = "ReconfigureMuxer:"
        for node in self.managedNodes:
            res += "\n- " + node
        return res
    
    def update_configurable_nodes(self):
            configurables = find_reconfigure_services()
            # creating client not yet managed
            for node in configurables:
                if node in self.clients.keys():
                    continue
                if len(self.managedNodes) == 0 or node in self.managedNodes:
                    self.add_client(node)

            # deleting client which node no longer exists
            # (is this necessary ?)
            for node in self.clients.keys():
                if node not in configurables:
                    self.delete_client(node)

    def add_client(self, node):
        with self.lock:
            if node in self.clients.keys():
                return
            # Creating new reconfigure client
            print("New reconfigure client :", node)
            self.clients[node] = ReconfClient(node, timeout=20,
                config_callback=partial(self.main_callback, node))
            # This check allows to keep old registered callbacks instead of erasing
            # them in case a node gets restarted
            if node not in self.configCallbacks.keys():
                self.configCallbacks[node] = {}

    def delete_client(self, node):
        with self.lock:
            print("Deleting client :", node)
            del self.clients[node]

    def main_callback(self, node, config):
        callbacks = []
        with self.lock:
            callbacks = [c for c in self.configCallbacks[node].values()]
        for c in callbacks:
            c(config)
    
    def add_callback(self, node, callback):
        node = ReconfigureMuxer.format_node_name(node)
        callbackId = uuid4()
        if node not in self.configCallbacks.keys():
            self.configCallbacks[node] = {}
        self.configCallbacks[node][callbackId] = callback
        # this is to quickly retrieve a callback in self.configCallbacks
        self.callbackMap[callbackId] = node 
        return callbackId

    def delete_callback(self, callbackId):
        if not callbackId in self.callbackMap:
            return
        del self.configCallbacks[self.callbackMap[callbackId]][callbackId]

rospy.init_node('reconfigure_muxer', anonymous=True, disable_signals=True)

muxer = ReconfigureMuxer(['oculus_sonar', 'ueye_cam_nodelet'])

def oculus_callback(config):
    print("Got oculus config:")
    print(config)
muxer.add_callback('oculus_sonar', oculus_callback)

def camera_callback(config):
    print("Got camera config:")
    print(config)
muxer.add_callback('ueye_cam_nodelet', camera_callback)

while 1:
    muxer.update_configurable_nodes()
    time.sleep(0.5)


