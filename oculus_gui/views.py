from django.shortcuts import render
from django.http import HttpResponse

from .cache import cache

def index(request):
    return render(request, "index.html")

def sonar_monitor(request):
    return render(request, "sonar_monitor.html")

def get_cached_data(request, dataUuid):
    if not request.method == 'GET':
        return HttpResponse(status=400)

    # print("Getting data :", dataUuid)
    data = cache.get(dataUuid)
    if data is None:
        return HttpResponse(status=404)
    return HttpResponse(content=data, status=200)

