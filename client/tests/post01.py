#! /usr/bin/python

import sys
sys.path.append('../')

from narval_monitor import Session

session = Session(rootUrl='http://127.0.0.1:8000/payload_monitor')

def post():
    data = bytearray(b'hello there')
    response = session.post('/post_status_test', data)
    print(response)
post()
