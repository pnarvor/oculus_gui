from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path('sonar_monitor', views.sonar_monitor, name='sonar_monitor'),

    path('get_cached_data/<str:dataUuid>', views.get_cached_data, name='get_cached_data'),
]


