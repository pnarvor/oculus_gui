import uuid
import threading
from collections import deque

class Cache:

    def __init__(self, minElements = 50, maxElements=70):
        self.data   = {}
        self.idList = deque()
        self.minElements = minElements
        self.maxElements = maxElements
        self.lock = threading.Lock();

    def insert(self, item):
        with self.lock:
            key = str(uuid.uuid4())
            self.data[key] = item
            self.idList.append(key)

            self.check_size()
            
            # print("Inserted :",key, ",", len(item), "bytes.")
            return key

    def check_size(self):
            if len(self.idList) > self.maxElements:
                while len(self.idList) > self.minElements:
                    del self.data[self.idList.popleft()]
    
    def get(self, key):
        with self.lock:
            if key in self.data:
                return self.data[key]
            else:
                return None


cache = Cache()
