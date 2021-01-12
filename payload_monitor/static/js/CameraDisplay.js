

class CameraDisplay
{
    constructor(canvas)
    {
        canvas.cameraDisplay = this;

        this.canvas = canvas;
        this.context = canvas.getContext('2d');
        this.context.fillStyle = "black";
        this.context.fillRect(0, 0, canvas.width, canvas.height);

        this.imageListener = new DataListener('/ws/subscribe/camera_image_raw/');
        this.imageListener.callbacks.push(this.image_callback.bind(this));

        this.busy   = false;
    }

    async image_callback(content)
    {
        // does this "mutex" really is secure ? (this function is
        // asynchronously called by the callback of a websocket.
        if(this.busy) {
            //console.log("Busy : ignoring data");
            return;
        }
        this.busy = true;
        
        //try {
            let metadata = JSON.parse(content.content.scalars);
            let rgbData = new Uint8ClampedArray(await content.fetch_cached_data('data'));
            // @#^$*!
            //let rgbaData = new Uint8ClampedArray(4*width*heigth);
            //for(let irgb = 0, irgba = 0; irgb < rgbData.length; irgb += 3) {
            //    rgbaData[irgba]     = rgbData[irgb + 2];
            //    rgbaData[irgba + 1] = rgbData[irgb + 1];
            //    rgbaData[irgba + 2] = rgbData[irgb];
            //    rgbaData[irgba + 3] = 255;
            //    irgba += 4;
            //}
            //let imageData = new ImageData(rgbaData, metadata.width);

            let subsampling = 4
            let width  = Math.floor(metadata.width  / subsampling);
            let height = Math.floor(metadata.height / subsampling);
            let rgbaData = new Uint8ClampedArray(4*width*height);
            for(let h = 0, irgb = 0; h < height; h++) {
                for(let w = 0; w < width; w++) {
                    let irgba = 4*(width * h + w);
                    let irgb  = 3*subsampling*(metadata.width*w + h)
                    
                    rgbaData[irgba]     = rgbData[irgb];
                    rgbaData[irgba + 1] = rgbData[irgb + 1];
                    rgbaData[irgba + 2] = rgbData[irgb + 2];
                    rgbaData[irgba + 3] = 255;
                }
            }
            let imageData = new ImageData(rgbaData, width);

            this.context.drawImage(await createImageBitmap(imageData),0,0);
        //}
        //finally {
            this.busy = false;
        //}
    }
};
