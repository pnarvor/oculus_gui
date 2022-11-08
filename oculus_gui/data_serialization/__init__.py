import json

from . import oculus_sonar

serializers = {}
serializers.update(oculus_sonar.serializers)

def serialize(serializerId, *args, **kwargs):
    if serializerId not in serializers.keys():
        return {}
    else:
        return serializers[serializerId][1](*args, **kwargs)



