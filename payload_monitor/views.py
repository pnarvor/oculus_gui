import json
import time

from django.shortcuts import render
from django.http import HttpResponse
from django.views.decorators.csrf import ensure_csrf_cookie, csrf_protect, csrf_exempt

from . import consumers

# from .cache import Cache
# cache = Cache()
from .cache import cache

@ensure_csrf_cookie
def status(request):
    return render(request, 'status.html')

@ensure_csrf_cookie
def render_page(request, page):
    print("Here", page)
    return render(request, page + '.html')

@ensure_csrf_cookie
def index(request):
    return render(request, 'index.html')

@ensure_csrf_cookie
def sonar_monitor(request):
    return render(request, 'sonar_monitor.html')

@ensure_csrf_cookie
def camera_monitor(request):
    return render(request, 'camera_monitor.html')

@csrf_protect
# @csrf_exempt
def post_data(request, topicName):
    if not request.method == 'POST':
        return HttpResponse(status=400)

    msg = {'topic' : topicName, 'type' : 'empty', 'scalars' : 'None', 'vectors' : 'None'}
    if 'scalars' in request.POST:
        msg['scalars'] = request.POST['scalars']
        msg['type']    = 'form_data'
    
    dataUuid = None
    if len(request.FILES) > 0:
        # Putting FILES in cache. Consumers are notified by the websocket that
        # new data is available can should specifically ask for it with a
        # request to get_cached_data.
        msg['type']    = 'cached_data'
        msg['vectors'] = {}
        for name, data in request.FILES.items():

            # raw_data = b'';
            # for chunk in data:
            #     raw_data += chunk
            # No need for a loop
            # dataUuid = cache.insert(raw_data)

            dataUuid = cache.insert(data.read())
            msg['vectors'][name] = {
                'data_uuid'         : dataUuid,
                'cache_request_uri' : '/payload_monitor/get_cached_data/'}
    
    # for socket in consumers.register.values():
        # socket.update(text_data=json.dumps(msg, ensure_ascii=True))
    if topicName in consumers.subscribers.keys():
        for socket in consumers.subscribers[topicName].values():
            socket.update(text_data=json.dumps(msg, ensure_ascii=True))

    response = HttpResponse(status=200)
    if dataUuid is not None:
        response['data_uuid'] = dataUuid
    return response

@csrf_protect
def get_cached_data(request, dataUuid):
    if not request.method == 'GET':
        return HttpResponse(status=400)

    print("Getting data :", dataUuid)
    data = cache.get(dataUuid)
    if data is None:
        return HttpResponse(status=404)
    return HttpResponse(content=data, status=200)


@csrf_exempt
def generic_post(request):
    print("Got post request")
    
    print("================ META\n", request.META)
    print("================ COOKIES\n", request.COOKIES)
    print("================ headers\n", request.headers)
    print("================ POST\n", request.POST)
    print("================ FILES\n", request.FILES)
    for f in request.FILES.items():
        print(f)
    # print("================ BODY\n", request.body)
    print("\n\n\n\n\n\n")

    return HttpResponse(content="POST ok", status=200)

