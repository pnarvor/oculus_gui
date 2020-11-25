
# python 2-3 compatibility imports
from __future__ import print_function

import requests
import json

class Session:

    """
    Session

    Manage communications with the django server (main use is to hold and
    transfer the CSRF token. (needed to identify the client on the server).
    """

    def print_response(r, verbose=True):
        print('status code  :',   r.status_code)
        print('content type :',   r.headers['content-type'])
        print('encoding     :',   r.encoding)
        if verbose:
            print('text         :\n', r.text, end='\n\n')
            if 'json' in r.headers['content-type']:
                print('json         :\n', r.json(), end='\n\n')

    def __init__(self, rootUrl='http://127.0.0.1:8000/', loginUrl=None):
        self.connect(rootUrl, loginUrl)


    def connect(self, rootUrl='http://127.0.0.1:8000/', loginUrl=None):
        self.rootUrl  = rootUrl
        self.loginUrl = loginUrl
        self.session  = requests.session()
        
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
    

    def post_message(self, url, metadata=None, raw_data=None):
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
        self.session.post(self.rootUrl + url, **post_data)

    def get(self, url):
        r = self.session.get(self.rootUrl + url)
        return r


