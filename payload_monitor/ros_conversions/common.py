

def from_time(msg):
    return {'scalars' : {'type'  : 'ros/time',
                         'secs'  : msg.secs,
                         'nsecs' : msg.nsecs},
            'vectors' : {}}

def from_Header(msg):
    return {'scalars' : {'type'     : 'std_msgs/Header',
                         'seq'      : msg.seq,
                         'stamp'    : from_time(msg.stamp),
                         'frame_id' : msg.frame_id},
            'vectors' : {}}


