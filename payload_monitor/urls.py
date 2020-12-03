from django.urls import path
from django.views.generic import RedirectView

from . import views

urlpatterns = [
    path('', RedirectView.as_view(url='status')),
    path('status', views.status, name='status'),

    path('view/<str:page>', views.render_page, name='generic_render'),

    path('array_tests', views.array_tests, name='array_tests'),
    path('webgl_test', views.webgl_test, name='webgl_test'),
    path('narval_display_test', views.narval_display_test, name='narval_display_test'),

    path('post_data', views.post_data, name='post_data'),
    path('get_cached_data/<str:dataUuid>', views.get_cached_data, name='get_cached_data'),
]


