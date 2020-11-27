from django.urls import path
from django.views.generic import RedirectView

from . import views

urlpatterns = [
    path('', RedirectView.as_view(url='status')),
    path('status', views.status, name='status'),

    path('generic_update', views.generic_update, name='generic_update'),

    path('array_tests', views.array_tests, name='array_tests'),

    path('webgl_test', views.webgl_test, name='webgl_test'),
]
