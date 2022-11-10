
import os

OculusLink = None

param = 0
if 'OCULUS_GUI_USE_ROS' in os.environ:
    param = int(os.environ['OCULUS_GUI_USE_ROS'])
if param == 1:
    # use ROS1
    print("Using ROS1 sonar client")
    from .oculus_link_ros1 import OculusLink
elif param == 2:
    # use ROS2
    print("Using ROS2 sonar client")
    from .oculus_link_ros2 import OculusLink
else:
    # don't use ros. Direct sonar connection
    from .oculus_link      import OculusLink



