from django.shortcuts import render
from django.http import HttpResponse

def index(request):
    return HttpResponse("Hello there ! Here is payload monitor index")


