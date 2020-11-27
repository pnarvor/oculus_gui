

class PingDisplay
{
    constructor(divContainer)
    {
        this.container     = divContainer;
        this.pingImageData = new ArrayBuffer(0);

        this.websocket = new WebSocket(
            'ws://' + window.location.host + '/ws/ping_display/');
        this.websocket.target = this;
        this.websocket.onmessage = function(e) {
            this.target.update(e);
        };

        this.updateCount = 0;


        console.log('Built PingDisplay');
    }

    async update(e)
    {
        //console.log(e.data);

        //console.log("Got data", this.updateCount);
        this.updateCount++;

        
        // this creates an untyped ArrayBuffer (hopefully view).
        // await Reponse needed because ?
        let buffer = await new Response(e.data).arrayBuffer();
        let data   = new Uint8Array(buffer);

        let metadataSize = new DataView(buffer).getUint32(0, true);
        let metadata = JSON.parse(new TextDecoder("utf-8").decode(
            data.subarray(4, 4 + metadataSize)));
        let raw_data = data.subarray(4 + metadataSize); // until the end
        
        this.update_display(metadata, raw_data);
    }
    
    update_display(metadata, raw_data)
    {
        let width  = metadata.nBeams;
        let height = metadata.nRanges;

        // allocating image data
        if(this.pingImageData.byteLength < 4*width*height) {
            this.pingImageData = new ArrayBuffer(4*width*height);
        }
        let imgData = new Uint8ClampedArray(this.pingImageData);
        
        let pingData;
        let bitShift;
        let gainOffset;
        if(metadata.fireMessage.flags & 0x2) {
            pingData = new Uint16Array(raw_data.subarray(metadata.imageOffset));
            bitShift = 8;
            gainOffset = 2;
        }
        else {
            pingData = new Uint8Array(raw_data.subarray(metadata.imageOffset));
            bitShift = 0;
            gainOffset = 4;
            console.log("Estimated gain depth :", (metadata.imageSize - width*height) / height);
        }
        if(!(metadata.fireMessage.flags & 0x4)) { // range gains are not sent
            console.log("gain is not sent");
            gainOffset = 0;
        }
        
        let idxIn  = 0;
        let idxOut = 0;
        for(let h = 0; h < height; h++) {
            idxIn += gainOffset;
            for(let w = 0; w < width; w++) {
                let value = pingData[idxIn];
                if(metadata.fireMessage.flags & 0x2) {
                    //value = 0x00ff & value;
                    value = value / 256;
                }
                //let value = pingData[idxIn] >> bitShift;
                imgData[idxOut]     = value;
                imgData[idxOut + 1] = value;
                imgData[idxOut + 2] = value;
                imgData[idxOut + 3] = 255;

                idxOut += 4;
                idxIn  += 1;
            }
        }
        console.log("idxIn :", idxIn);
        console.log("pingData size :", pingData.length);

        this.update_canvas(width, height, imgData);
    }

    async update_canvas(width, height, data) {

        let img = await createImageBitmap(new ImageData(data, width));
        //console.log(img);
        
        let canvas  = $('#img0')[0];
        canvas.width  = width;
        canvas.height = height;

        let context = canvas.getContext('2d');

        context.drawImage(img, 0, 0);
        //console.log(context);
    }
};

$(document).ready(function() {
    $('#status_div')[0].pingDisplay = new PingDisplay($('#status_div')[0]);
});
