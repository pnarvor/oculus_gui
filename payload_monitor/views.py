import json

from django.shortcuts import render
from django.http import HttpResponse
from django.views.decorators.csrf import ensure_csrf_cookie, csrf_protect

from . import consumers

from .cache import Cache
cache = Cache()

@ensure_csrf_cookie
def status(request):
    return render(request, 'status.html')

@ensure_csrf_cookie
def array_tests(request):
    return render(request, 'array_tests.html')

@ensure_csrf_cookie
def webgl_test(request):
    return render(request, 'webgl_test.html')

@ensure_csrf_cookie
def narval_display_test(request):
    return render(request, 'narval_display_test.html')

@csrf_protect
def post_data(request):
    if not request.method == 'POST':
        return HttpResponse(status=400)
    
    msg = {'type' : 'empty', 'metadata' : 'None'}
    if 'metadata' in request.POST:
        msg['metadata'] = request.POST['metadata']
        msg['type']     = 'form_data'
    
    dataUuid = None
    if 'raw_data' in request.FILES:
        raw_data = b'';
        count = 0
        for chunk in request.FILES['raw_data']:
            raw_data += chunk
            count += 1
        msg['type'] = 'cached_data'
        dataUuid = cache.insert(raw_data)
        msg['data_uuid']         = dataUuid
        msg['cache_request_url'] = '/payload_monitor/get_cached_data/'

    for socket in consumers.register.values():
        socket.update(text_data=json.dumps(msg))

    response = HttpResponse(status=200)
    if dataUuid is not None:
        response['data_uuid'] = dataUuid
    return response

@csrf_protect
def get_cached_data(request, dataUuid):
    if not request.method == 'GET':
        return HttpResponse(status=400)
    data = cache.get(dataUuid)
    if data is None:
        return HttpResponse(status=404)
    return HttpResponse(content=data, status=200)



