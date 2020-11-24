from django.shortcuts import render
from django.http import HttpResponse

def index(request):
    return render(request, 'base.html')

def status(request):
    return HttpResponse("This is status.")



