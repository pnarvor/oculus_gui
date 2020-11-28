from django.shortcuts import render
from django.http import HttpResponse
from django.views.decorators.csrf import ensure_csrf_cookie, csrf_protect

from . import consumers

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
def generic_update(request):
    if not request.method == 'POST':
        return HttpResponse(400)
    
    if 'raw_data' not in request.FILES and 'metadata' in request.POST:
        # if no binary data, sending metadata data as "text"
        metadata = request.POST['metadata']
        # Have to do it this way for reasons explained below
        for socket in consumers.register.values():
            socket.update(text_data=metadata)
    elif 'raw_data' in request.FILES:
        # if binary data, sending metadata in the binary blob.
        if 'metadata' in request.POST:
            metadata = request.POST['metadata']
            # print('Metadata size :', len(metadata))
            raw_data = (len(metadata)).to_bytes(4, byteorder='little',
                                                    signed=False)
            raw_data += metadata.encode(encoding='ascii')
        else:
            # print('Metadata size :', 0)
            raw_data = (0).to_bytes(4, byteorder='little', signed=False)

        # metadata = request.POST['metadata']
        # print("metadata type :", type(metadata))
        # raw_data = b''
        # print('raw_data type :', type(raw_data))
        # print('encoded  type :', type(metadata.encode(encoding='ascii')))
        # print('string  size :', len(metadata))
        # print('encoded size :', len(metadata.encode(encoding='ascii')))
        count = 0
        for chunk in request.FILES['raw_data']:
            raw_data += chunk
            count += 1
        # print('Chunk count : ', count)
        for socket in consumers.register.values():
            socket.update(bytes_data=raw_data)
    
    # for socket in consumers.register.values():
    #     # Not possible. The websocket will ignore bytes_data 
    #     # if text_data is not None
    #     socket.update(text_data=metadata, bytes_data=raw_data)

    return HttpResponse(200)


