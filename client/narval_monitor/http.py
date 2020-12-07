from __future__ import print_function

import time

from pprint import pformat

from twisted.internet.defer import Deferred, DeferredList
from twisted.internet.protocol import Protocol
from twisted.web.client import Agent, HTTPConnectionPool, RedirectAgent, CookieAgent, readBody
from twisted.web.http_headers import Headers
from twisted.python import log, compat
from twisted.python.url import URL

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

        self.rootUrl = URL(scheme=u'http', host=unicode(self.host),
                           port=self.port).asURI()
        if rootUrl is not None:
            self.rootUrl = self.rootUrl.click(unicode(rootUrl)).asURI()
        # This url is where to fetch the csrf token
        if loginUrl is None:
            self.loginUrl = self.rootUrl
        else:
            self.loginUrl = self.rootUrl.click(unicode(loginUrl)).asURI()

        # Client related code
        self.pool      = HTTPConnectionPool(self.reactor)
        self.cookieJar = compat.cookielib.CookieJar()
        self.agent     = CookieAgent(RedirectAgent(Agent(self.reactor, pool=self.pool)),
                                     self.cookieJar)
        self.csrf = None

        self.connected  = False
        self.connectionCbChain = None
        self.errorCount = 0
        self.maxErrorCount = 2

    def __str__(self):
        return 'HttpSession :' \
               + '\n- host  : ' + self.rootUrl.asText() \
               + '\n- login : ' + self.loginUrl.asText() 

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
        print("Succes, error count set to 0")
        return response
    
    def connect(self):
        if self.connectionCbChain is not None:
            # if self.connectionCbChain is not None we alreading are in the
            # process of connecting.
            return Deferred()
        self.initiate_connection()

    def initiate_connection(self):
        self.connectionCbChain = self.agent.request('GET', bytes(self.loginUrl.asText()))
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

    def request(self, method, uri, headers=None, bodyProducer=None):
        uri = bytes(self.rootUrl.click(unicode(uri)).asText())
        print('New request', uri)
        if headers is None:
            headers = Headers()
        headers.addRawHeader('X-CSRFtoken', self.csrf)
        d = self.agent.request(method=method, uri=uri, headers=headers, 
                               bodyProducer=bodyProducer)

        d.addCallback(self.success_counter)
        d.addErrback(self.error_counter)
        return d
