from sensor_msgs.msg import Image as ImageMsg

def from_Image(msg):
    return {'scalars' : {'width'       : msg.width,
                         'height'      : msg.height,
                         'encoding'    : msg.encoding,
                         'isbigendian' : msg.is_bigendian,
                         'step'        : msg.step},
            'vectors' : {'data' : ('data', msg.data)}}

# ros_message_types = {'sensor_msgs/Image' : ImageMsg}
# ros_converters = {ImageMsg, from_Image}

ros_converters = {'sensor_msgs/Image' : (ImageMsg, from_Image)}
