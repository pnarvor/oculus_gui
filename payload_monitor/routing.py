from django.urls import re_path

from . import consumers
from . import reconfigure_bridge
from . import ros_bridge

websocket_urlpatterns = [
    # re_path(r'ws/ping_display/$', consumers.PingConsumer.as_asgi()),
    re_path(r'ws/subscribe/$', consumers.DataUpdater.as_asgi()),
    re_path(r'ws/subscribe/(?P<topicName>[\w\-]+)/$',
            consumers.TopicBridge.as_asgi()),
    # re_path(r'ws/subscribe/(?P<argument>[\w\-]+)/$',consumers.DataUpdater.as_asgi()),

    re_path(r'ws/subscribe_test/(?P<topicName>[\w\-]+)/$',
            consumers.TopicBridge_test.as_asgi()),

    re_path(r'ws/ros_subscribe/(?P<topicName>[\w\-]+)/(?P<topicType>[\w\-]+)/$',
            ros_bridge.Subscriber.as_asgi()),

    re_path(r'ws/websocket_test/$', consumers.WebSocketTest.as_asgi()),

    re_path(r'ws/reconfigure_bridge/(?P<nodeName>[\w\-]+)/$',
            reconfigure_bridge.ReconfigureBridge.as_asgi()),
    re_path(r'ws/reconfigure_client/(?P<nodeName>[\w\-]+)/$',
            reconfigure_bridge.ReconfigureClient.as_asgi()),
]
# websocket_urlpatterns = [
#     # re_path(r'ws/ping_display/$', consumers.PingConsumer.as_asgi()),
#     re_path(r'ws/subscribe/$', consumers.DataUpdater),
#     re_path(r'ws/subscribe/(?P<topicName>[\w\-]+)/$',
#             consumers.TopicBridge),
#     # re_path(r'ws/subscribe/(?P<argument>[\w\-]+)/$',consumers.DataUpdater.as_asgi()),
# 
#     re_path(r'ws/websocket_test/$', consumers.WebSocketTest),
# 
#     re_path(r'ws/reconfigure_bridge/(?P<nodeName>[\w\-]+)/$',
#             reconfigure_bridge.ReconfigureBridge),
#     re_path(r'ws/reconfigure_client/(?P<nodeName>[\w\-]+)/$',
#             reconfigure_bridge.ReconfigureClient),
# ]
