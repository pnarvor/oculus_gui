import json

from . import Session

class PayloadSession:

    def __init__(self, rootUrl, loginUrl=None):
        self.session = Session(rootUrl, loginUrl)

    def post_message(self, metadata=None, raw_data=None):
        post_data = {}
        if metadata is not None:
            post_data['data'] = {'metadata' : json.dumps(metadata)}
        if raw_data is not None:
            post_data['files'] = {'raw_data' : ('data', raw_data)}
        self.session.session.post(self.session.rootUrl + '/generic_update',
                                  **post_data)

    def post_sonar_ping(self, pingMsg):
        files = {'ping_data' : ('data', pingMsg.data)}
        self.session.session.post(self.session.rootUrl + '/ping_update',
                                  files=files)
