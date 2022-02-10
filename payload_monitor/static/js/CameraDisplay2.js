
class CameraDisplay2
{
    constructor(imageDiv, topicName, topicType)
    {
        this.imageDiv = imageDiv;

        this.imageListener = new RosTopicListener(topicName, topicType);
        this.imageListener.callbacks.push(this.image_callback.bind(this));

        this.busy   = false;
        this.currentObjectURL = undefined;
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
        
        try {
            let metadata = JSON.parse(content.content.scalars);
            //let data = new Uint8ClampedArray(await content.fetch_cached_data('data'));
            let data = new Blob([new Uint8ClampedArray(await content.fetch_cached_data('data'))]);
            
            //if(this.currentObjectURL !== undefined) {
            //    window.URL.revokeObjectURL(this.currentObjectURL);
            //}
            window.URL.revokeObjectURL(this.currentObjectURL);
            this.currentObjectURL = window.URL.createObjectURL(data);
            this.imageDiv.src = this.currentObjectURL;
            
            // console.log(metadata);
            // console.log(data);
            // console.log(window.URL.createObjectURL(data));
        }
        finally {
          this.busy = false;
        }
    }
};
