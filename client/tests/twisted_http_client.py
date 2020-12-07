from __future__ import print_function

import time
from pprint import pformat

from twisted.internet import reactor
from twisted.internet.defer import Deferred

from narval_monitor.http import HttpSession
from narval_monitor.http import IgnoreBody
from narval_monitor.http import PrintBody

session = HttpSession(reactor, '127.0.0.1', rootUrl='payload_monitor/')
print(session)
session.connect()

def new_request():
    d = session.request('POST', 'generic_post')
    d.addCallback(test_callback)
    d.addErrback(test_errback)

def test_callback(response):
    print("Reponse :", response)

    print('Response code:', response.code)
    print('Response version:', response.version)
    print('Response code:', response.code)
    print('Response phrase:', response.phrase)
    print('Response headers:')
    print(pformat(list(response.headers.getAllRawHeaders())))
    # print('Cookies :', len(cookieJar))
    # for cookie in cookieJar:
    #     print(cookie)
    finished = Deferred()
    response.deliverBody(PrintBody(finished))

    reactor.callLater(1.0, new_request)
    return finished

def test_errback(failure):
    # print("Got error")
    print("Got error :", failure)
    reactor.callLater(1.0, new_request)
reactor.callLater(0.0, new_request)

reactor.run()


