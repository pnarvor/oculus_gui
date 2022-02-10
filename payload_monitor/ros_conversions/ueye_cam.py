from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

def from_Image(msg):
    return {'scalars' : {'width'       : msg.width,
                         'height'      : msg.height,
                         'encoding'    : msg.encoding,
                         'isbigendian' : msg.is_bigendian,
                         'step'        : msg.step},
            'vectors' : {'data' : ('data', msg.data)}}

# ros_message_types = {'sensor_msgs/Image' : ImageMsg}
# ros_converters = {ImageMsg, from_Image}

def from_CompressedImage(msg):
    return {'scalars' : {'format': msg.format},
            'vectors' : {'data'  : ('data', msg.data)}}

ros_converters = {'sensor_msgs/Image'           : (Image, from_Image),
                  'sensor_msgs/CompressedImage' : (CompressedImage, from_CompressedImage)}
