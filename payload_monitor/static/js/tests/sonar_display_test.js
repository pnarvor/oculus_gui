
class SonarDisplay extends Display
{
    constructor(canvas)
    {
        super(canvas);

        this.pingRenderer = new SonarRenderer(this.gl);
        this.add_renderer(this.pingRenderer);

        //this.pingListener = new DataListener();
        this.pingListener = new DataListener('/ws/subscribe/test/');
        this.pingListener.callbacks.push(this.ping_callback.bind(this));

        this.busy = false;
    }

    async ping_callback(content)
    {
        // does this "mutex" really is secure ? (this function is
        // asynchronously called by the callback of a websocket.
        if(this.busy) {
            console.log("Busy : ignoring data");
            return;
        }
        this.busy = true;
        
        try {
            let metadata = JSON.parse(content.content.metadata);
            let data = new Uint8Array(await content.fetch_cached_data());

            this.pingRenderer.set_ping_data(metadata, data.subarray(metadata.imageOffset));
        }
        finally {
            this.busy = false;
        }
        this.draw();
    }
};


$(document).ready(function() {
    let display = new SonarDisplay($("#main_display")[0]);
    window.onresize = function() {
        $("#main_display")[0].naglContext
            .resize(window.innerWidth, window.innerHeight);
    };
    window.onresize();
});
