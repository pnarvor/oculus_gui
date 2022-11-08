
from django.urls import re_path

from . import consumers

websocket_urlpatterns = [
    re_path(r"ws/sonar_data/$", consumers.OculusConsumer.as_asgi()),
]
