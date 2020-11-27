

class PingDisplay
{
    constructor(divContainer)
    {
        this.container      = divContainer;
        
        let width  = 256;
        let height = 256;
        this.dataBuffer = new ArrayBuffer(width*height*4);

        let data = new Uint8ClampedArray(this.dataBuffer);
        for(let h = 0; h < height; h++) {
            for(let w = 0; w < width; w++) {
                let value = 255*((w + h) & 0x1);
                data[4*(width*h + w)]     = value;
                data[4*(width*h + w) + 1] = value;
                data[4*(width*h + w) + 2] = value;
                data[4*(width*h + w) + 3] = 255;
            }
        }

        console.log("Update canvas");
        this.update_canvas(width, height, data);
    }
    
    async update_canvas(width, height, data) {

        let img = await createImageBitmap(new ImageData(data, width));
        console.log(img);
        
        let canvas  = $('#img0')[0];
        canvas.width  = width;
        canvas.height = height;

        let context = canvas.getContext('2d');

        context.drawImage(img, 0, 0);
    }
};

$(document).ready(function() {
    $('#status_div')[0].pingDisplay = new PingDisplay($('#status_div')[0]);
});
