from __future__ import print_function

import time
import json

from pprint import pformat

from twisted.internet.defer import Deferred, DeferredList
from twisted.internet.protocol import Protocol
from twisted.web.client import Agent, HTTPConnectionPool, CookieAgent
# from twisted.web.client import RedirectAgent
from twisted.web.client import BrowserLikeRedirectAgent as RedirectAgent
from twisted.web.http_headers import Headers
from twisted.python import log, compat
from twisted.python.url import URL

from treq.client import HTTPClient

class IgnoreBody(Protocol):
    def __init__(self, deferred):
        self.deferred = deferred

    def dataReceived(self, bytes):
        pass

    def connectionLost(self, reason):
        self.deferred.callback(None)

class PrintBody(Protocol):
    def __init__(self, deferred):
        self.deferred = deferred

    def dataReceived(self, bytes):
        print("Body :", bytes)

    def connectionLost(self, reason):
        self.deferred.callback(None)


class HttpSession:
    response = 0
    def __init__(self, reactor, host, port=8000, rootUrl=None, loginUrl=None):
        self.reactor   = reactor
        self.host      = host
        self.port      = 8000
        
        # the way url and handled is messed up... Cannot follow redictions
        # self.rootUrl = URL(scheme=u'http', host=unicode(self.host),
        #                    port=self.port).asURI()
        
        self.rootUrl = 'http://' + host + ':' + str(port) + '/'

        # This url is where to fetch the csrf token
        if loginUrl is None:
            if rootUrl is not None:
                self.loginUrl = self.child_path(rootUrl)
            else:
                self.loginUrl = self.rootUrl
        else:
            self.loginUrl = self.child_path(loginUrl)
        # all next request will use self.rootUrl as prefix
        if rootUrl is not None:
            self.rootUrl = self.child_path(rootUrl)


        # Client related code
        self.pool      = HTTPConnectionPool(self.reactor)
        self.cookieJar = compat.cookielib.CookieJar()
        self.agent     = CookieAgent(RedirectAgent(Agent(self.reactor, pool=self.pool)),
                                     self.cookieJar)
        self.session   = HTTPClient(self.agent)
        self.csrf = None

        self.connected  = False
        self.connectionCbChain = None
        self.errorCount = 0
        self.maxErrorCount = 2

    def __str__(self):
        return 'HttpSession :' \
               + '\n- host  : ' + self.rootUrl \
               + '\n- login : ' + self.loginUrl 
               # + '\n- host  : ' + self.rootUrl.asText() \
               # + '\n- login : ' + self.loginUrl.asText() 
    
    def child_path(self, path):
        # res = self.rootUrl
        # for p in [unicode(p) for p in path.split('/') if len(p) > 0]:
        #     res = res.child(p)
        # res.child(u'')

        # have to do this by hand because hyperlink.URL is messed up (cannot
        # add a simple trailing slash)
        if self.rootUrl[-1] == '/':
            res = self.rootUrl[:-1]
        else:
            res = self.rootUrl
        for p in ['/' + p for p in path.split('/') if len(p) > 0]:
            res += p
        if path[-1] == '/':
            res += '/'
        return res

    def error_counter(self, failure):
        self.errorCount += 1
        print("Got error :", failure)
        print("Error count :", self.errorCount)
        if self.errorCount > self.maxErrorCount:
            self.connected = False
            self.connect()
        return failure
        # return None

    def success_counter(self, response):
        self.errorCount = 0
        # print("Succes, error count set to 0")
        return response
    
    def connect(self):
        if self.connectionCbChain is not None:
            # if self.connectionCbChain is not None we alreading are in the
            # process of connecting.
            return Deferred()
        self.initiate_connection()

    def initiate_connection(self):
        # self.connectionCbChain = self.agent.request('GET', bytes(self.loginUrl.asText()))
        self.connectionCbChain = self.agent.request('GET', self.loginUrl)
        self.connectionCbChain.addCallback(self.initiate_callback, self.cookieJar)
        self.connectionCbChain.addErrback(self.initiate_error)

    def initiate_callback(self, response, cookieJar):
        finished = Deferred()
        try:
            # Trying to get csrf cookie
            for cookie in cookieJar:
                if cookie.name == 'csrftoken':
                    self.csrf = cookie.value
            if self.csrf is None:
                print("Got csrf error")
                errString = "Connection error : Got response from server but"\
                          + "could not retrieve csrftoken."\
                          + '\n- Response code    : ' + str(response.code)\
                          + '\n- Response version : ' + str(response.version)\
                          + '\n- Response phrase  : ' + str(response.phrase) \
                          + '\n- Response headers :'\
                          + pformat(list(response.headers.getAllRawHeaders()))
                raise RuntimeError(errString)
            self.connectionCbChain = None
            self.connected = True
            self.errorCount = 0
            print("Connection to", self.rootUrl, "successful.")
        finally:
            # Always have to read body. (This callback is called as soon as the
            # headers are received, before the body is received. If the body is not
            # read, this conneciton never finishes).
            response.deliverBody(IgnoreBody(finished))
        return finished

    def initiate_error(self, failure):
        self.errorCount += 1
        print("Failed to initiate connection with server (error count : "
              + str(self.errorCount) + "). Retrying...")
        print("    (" + failure.getErrorMessage() + ")")

        # Retrying a connection with a 1s delay
        self.reactor.callLater(1.0, self.initiate_connection)
        return None # end of error chain

    def post(self, *args, **kwargs):
        if self.csrf is None or not self.connected:
            return Deferred()
        if 'headers' not in kwargs.keys():
            kwargs['headers'] = {}
        kwargs['headers']['X-CSRFtoken'] = self.csrf
        d = self.session.post(*args, **kwargs)
        d.addCallback(self.success_counter)
        d.addErrback(self.error_counter)
        d.addErrback(self.print_failure)
        return d


    def post_message_callback(self, response):
        print(response)

    def post_message_error(self, failure):
        print(failure)

    def print_failure(self, failure):
        print("Printing failure")
        print(failure)
