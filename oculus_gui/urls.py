from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path('sonar_monitor', views.sonar_monitor, name='sonar_monitor'),
]
