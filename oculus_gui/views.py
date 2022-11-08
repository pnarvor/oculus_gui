from django.shortcuts import render

def index(request):
    return render(request, "index.html")

def sonar_monitor(request):
    return render(request, "sonar_monitor.html")


