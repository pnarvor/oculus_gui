from .common import from_Header

from hemisphere_v500.msg import StampedString

def from_StampedString(msg):
    return {'scalars' : {'type'   : 'hemisphere_v500/StampedString',
                         'header' : from_Header(msg.header),
                         'data'   : msg.data},
            'vectors' : {}}

ros_converters = {'hemisphere_v500/StampedString' : (StampedString, from_StampedString) }
