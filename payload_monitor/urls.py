from django.urls import path
from django.views.generic import RedirectView

from . import views

urlpatterns = [
    path('', RedirectView.as_view(url='status')),
    path('status', views.status, name='status'),

    path('post_status_test', views.post_status_test, name='post_status_test'),
]
