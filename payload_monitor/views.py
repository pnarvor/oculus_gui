from django.shortcuts import render
from django.http import HttpResponse

def status(request):
    return render(request, 'status.html')



