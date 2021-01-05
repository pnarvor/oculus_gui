

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
            let data = new Uint8ClampedArray(await content.fetch_cached_data('data'));
            let imageData = new ImageData(data, metadata.width);
            console.log(imageData);
            this.context.drawImage(await createImageBitmap(imageData),0,0);
            console.log(metadata);
        //}
        //finally {
            this.busy = false;
        //}
    }
};
