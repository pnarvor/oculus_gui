from django.shortcuts import render
from django.http import HttpResponse
from django.views.decorators.csrf import ensure_csrf_cookie, csrf_protect

from . import consumers

@ensure_csrf_cookie
def status(request):
    return render(request, 'status.html')

@csrf_protect
def status_update(request):
    if not request.method == 'POST':
        return HttpResponse(400)

    print('\n\n\n')
    for key,value in request.META.items():
        print(key, ":", value)
    print('\n\n\n')

    for key,value in request.FILES.items():
        print(key, ":", type(value))
        # value.open()
        for chunk in value.chunks():
            print(chunk)

    print('\n\n\n')
    print("Type    :", type(request))
    print("COOKIES :", request.COOKIES)
    print("GET     :", request.GET)
    print("FILES   :", request.FILES)
    print("POST    :", request.POST)
    print('\n\n\n')

    return HttpResponse(200)


@csrf_protect
def ping_update(request):
    if not request.method == 'POST':
        return HttpResponse(400)

    # print('\n\n\n')
    # for key,value in request.META.items():
    #     print(key, ":", value)
    # print('\n\n\n')
    
    # for key,value in request.FILES.items():
    #     print(key, ":", type(value))
    #     # value.open()
    #     # for chunk in value.chunks():
    #     #     print(chunk)

    # print('\n\n\n')
    # print("Type    :", type(request))
    # print("COOKIES :", request.COOKIES)
    # print("GET     :", request.GET)
    # print("FILES   :", request.FILES)
    # print("POST    :", request.POST)
    # print('\n\n\n')

    ping_data = b''
    for chunk in request.FILES['ping_data']:
        ping_data += chunk
    # print(ping_data)

    for socket in consumers.register.values():
        socket.update(ping_data)

    return HttpResponse(200)

@csrf_protect
def post_status_test(request):
    if not request.method == 'POST':
        return HttpResponse(400)

    print('\n\n\n')
    for key,value in request.META.items():
        print(key, ":", value)
    print('\n\n\n')

    for key,value in request.FILES.items():
        print(key, ":", type(value))
        # value.open()
        for chunk in value.chunks():
            print(chunk)

    print('\n\n\n')
    print("Type    :", type(request))
    print("COOKIES :", request.COOKIES)
    print("GET     :", request.GET)
    print("FILES   :", request.FILES)
    print("POST    :", request.POST)
    print('\n\n\n')

    return HttpResponse(200)
