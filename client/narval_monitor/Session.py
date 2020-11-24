
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
    

    def post(self, url, data):
        files = {'file' : ('data', data)}
        r = self.session.post(self.rootUrl + url, files=files)
        return r
    

    def get(self, url):
        r = self.session.get(self.rootUrl + url)
        return r


