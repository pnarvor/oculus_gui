from . import Session

class PayloadSession:

    def __init__(self, rootUrl, loginUrl=None):
        self.session = Session(rootUrl, loginUrl)


    def post_sonar_ping(self, pingMsg):
        files = {'ping_data' : ('data', pingMsg.data)}
        self.session.session.post(self.session.rootUrl + '/ping_update',
                                  files=files)
