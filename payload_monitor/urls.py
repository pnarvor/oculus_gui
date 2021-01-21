from django.urls import path
from django.views.generic import RedirectView

from . import views

urlpatterns = [
    # path('', RedirectView.as_view(url='status')),
    path('', RedirectView.as_view(url='index')),
    path('status', views.status, name='status'),

    path('index',          views.index,          name='index'),
    path('sonar_monitor',  views.sonar_monitor,  name='sonar_monitor'),
    path('camera_monitor', views.camera_monitor, name='camera_monitor'),

    path('view/<str:page>', views.render_page, name='generic_render'),

    path('post_data/<str:topicName>', views.post_data, name='post_data'),
    path('get_cached_data/<str:dataUuid>', views.get_cached_data, name='get_cached_data'),

    path('generic_post', views.generic_post, name='generic_post'),
]


