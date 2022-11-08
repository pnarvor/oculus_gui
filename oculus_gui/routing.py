
from django.urls import re_path

from . import consumers

websocket_urlpatterns = [
    re_path(r"ws/sonar_data/$",         consumers.OculusConsumer.as_asgi()),
    re_path(r"ws/reconfigure_client/$", consumers.ReconfigureConsumer.as_asgi()),
]
