from django.urls import re_path

from . import consumers

websocket_urlpatterns = [
    # re_path(r'ws/ping_display/$', consumers.PingConsumer.as_asgi()),
    re_path(r'ws/subscribe/$', consumers.DataUpdater.as_asgi()),
]
