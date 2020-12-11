
# python 2-3 compatibility imports
from __future__ import print_function

import requests
import time
from requests.exceptions import ConnectionError
import json

class Session:

    """
    Session

    Manage communications with the django server (main use is to hold and
    transfer the CSRF token. (needed to identify the client on the server).
    """

    def print_response(r, verbose=True):
        print('status code  :',   r.status_code)
        # Getting csrf token
        if self.loginUrl is None:
            self.loginUrl = self.rootUrl
        self.session.get(self.loginUrl)
        try:
            self.csrfToken = self.session.cookies['csrftoken']
        except KeyError:
            raise RuntimeError("Could not get csrf token from "+self.loginUrl
                               + ". Is the server running ?")
        self.session.headers.update({'X-CSRFToken': self.csrfToken})
        print("Connection to",  self.rootUrl, "succesful.")
        print('content type :',   r.headers['content-type'])
        print('encoding     :',   r.encoding)
        if verbose:
            print('text         :\n', r.text, end='\n\n')
            if 'json' in r.headers['content-type']:
                print('json         :\n', r.json(), end='\n\n')

    def __init__(self, rootUrl='http://127.0.0.1:8000/', loginUrl=None):
        self.connected = False
        print(rootUrl)
        self.connect(rootUrl, loginUrl)

    def connect(self, rootUrl='http://127.0.0.1:8000/', loginUrl=None):
        self.rootUrl  = rootUrl
        self.loginUrl = loginUrl
        self.session  = requests.session()
        self.connectionErrorCount = 0
        self.try_connection()
    
    def try_connection(self, maxAttempts = -1):
        # Getting csrf token
        if self.loginUrl is None:
            self.loginUrl = self.rootUrl
        print("HEre")
        print(self.rootUrl)
        print(self.loginUrl)
        attempts = 0
        self.connected = False
        while 1:
            try:
                print(self.loginUrl)
                self.session.get(self.loginUrl, timeout=3.0)
                self.csrfToken = self.session.cookies['csrftoken']
                break
            except KeyError:
                print("Could not get csrf token from "+self.loginUrl+
                      ". Is the server running ?")
                time.sleep(0.5)
            except ConnectionError as e:
                print("ConnectionError (attemp "+str(attempts)+"),  retrying... ", e)
                time.sleep(0.5)
            finally:
                attempts += 1
                if maxAttempts > 0 and attempts >= maxAttempts:
                    raise ConnectionError("Connection Failure (max attemps reached)")
        self.connected = True
        self.session.headers.update({'X-CSRFToken': self.csrfToken})
        print("Connection to",  self.rootUrl, "succesful.")

    def child_path(self, path):
        if self.rootUrl[-1] == '/':
            res = self.rootUrl[:-1]
        else:
            res = self.rootUrl
        for p in ['/' + p for p in path.split('/') if len(p) > 0]:
            res += p
        if path[-1] == '/':
            res += '/'
        return res

    def post_message(self, url, metadata=None, raw_data=None, timeout=5.0):
        if not self.connected:
            return
        # Here both the metadata and data of the message will be sent together
        # to the server. However, in django, if metadata is present then
        # raw_data will be ignored. (no easy fix possible)
        post_data = {}
        if metadata is not None:
            # For json data to arrive in POST field in django, it must be
            # encasulated in a regular form (= python dict which will be
            # converted to an http form. Otherwise it will end up in the
            # message body.
            post_data['data'] = {'metadata' : json.dumps(metadata, ensure_ascii=True)}
        if raw_data is not None:
            # Sending raw (binary) via the HTTP FILES field.
            post_data['files'] = {'raw_data' : ('data', raw_data)}
        return self.session.post(self.rootUrl + url, timeout=timeout, **post_data)

    def get(self, url):
        if not self.connected:
            return
        r = self.session.get(self.rootUrl + url)
        return r
    
    # def post(self, url, post_data):
    def post(self, *args, **kwargs):
        if not self.connected:
            return
        # Here both the metadata and data of the message will be sent together
        # to the server. However, in django, if metadata is present then
        # raw_data will be ignored. (no easy fix possible)
        return self.session.post(*args, **kwargs)

