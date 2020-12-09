from __future__ import print_function

import time
import json

from pprint import pformat

from twisted.internet.defer import Deferred, DeferredList
from twisted.internet.protocol import Protocol
from twisted.web.client import Agent, HTTPConnectionPool, RedirectAgent, CookieAgent, readBody
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

        self.rootUrl = URL(scheme=u'http', host=unicode(self.host),
                           port=self.port).asURI()
        # This url is where to fetch the csrf token
        if loginUrl is None:
            self.loginUrl = self.rootUrl
        else:
            # self.loginUrl = self.rootUrl.child(unicode(loginUrl)).asURI()
            self.loginUrl = self.child_path(loginUrl)
        # all next request will use self.rootUrl as prefix
        if rootUrl is not None:
            # self.rootUrl = self.rootUrl.child(unicode(rootUrl)).asURI()
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
               + '\n- host  : ' + self.rootUrl.asText() \
               + '\n- login : ' + self.loginUrl.asText() 
    
    def child_path(self, path):
        res = self.rootUrl
        for p in [unicode(p) for p in path.split('/') if len(p) > 0]:
            res = res.child(p)
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
        uri = bytes(self.child_path(uri).asText())
        if headers is None:
            headers = Headers()
        headers.addRawHeader('X-CSRFtoken', self.csrf)
        d = self.agent.request(method=method, uri=uri, headers=headers, 
                               bodyProducer=bodyProducer)

        d.addCallback(self.success_counter)
        d.addErrback(self.error_counter)
        return d

    def post(self, uri, data=None, files=None):
        uri = bytes(self.rootUrl.child(unicode(uri)).asText())
        uri = bytes(self.child_path(uri).asText())
        kwargs = {}
        if data is not None:
            kwargs['data'] = data
        if files is not None:
            kwargs['files'] = files
        return self.session.post(uri, **kwargs)

    def post_message(self, uri, metadata=None, raw_data=None, timeout=5.0):
        # uri = bytes(self.rootUrl.child(unicode(uri)).asText())
        uri = bytes(self.child_path(uri).asText())
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
        d = self.session.post(uri, **post_data)
        # d.addCallback(self.post_message_callback)
        d.addErrback (self.post_message_error)
        return d

    def post_message_callback(self, response):
        print(response)

    def post_message_error(self, failure):
        print(failure)

    def print_failure(self, failure):
        print(failure)
