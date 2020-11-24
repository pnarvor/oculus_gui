from django.urls import path
from django.views.generic import RedirectView

from . import views

urlpatterns = [
    path('', RedirectView.as_view(url='status')),
    path('status', views.status, name='status'),

    path('status_update', views.status_update, name='status_update'),
    path('ping_update', views.ping_update, name='ping_update'),

    path('post_status_test', views.post_status_test, name='post_status_test'),
]
