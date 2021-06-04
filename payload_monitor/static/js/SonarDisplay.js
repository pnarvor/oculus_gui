class SonarDisplay extends Display
{
    constructor(container)
    {
        console.log(container);
        super(container.getElementsByClassName("sonar-canvas")[0]);

        this.container = container

        this.pingRenderer = new SonarRenderer(this.gl);
        this.add_renderer(this.pingRenderer);
        this.gl.clearColor(this.pingRenderer.zeroColor[0],
                           this.pingRenderer.zeroColor[1],
                           this.pingRenderer.zeroColor[2],
                           this.pingRenderer.zeroColor[3]);

        this.pingListener = new RosTopicListener('/ping', 'oculus_sonar/OculusPing');
        this.pingListener.callbacks.push(this.ping_callback.bind(this));

        this.busy = false;
        
        // Binding buttons to SonarDisplay control methods
        this.vFlipButton = container.getElementsByClassName("sonar-vflip")[0];
        this.vFlipButton.onclick = 
            this.pingRenderer.vertical_flip.bind(this.pingRenderer);
        this.hFlipButton = container.getElementsByClassName("sonar-hflip")[0];
        this.hFlipButton.onclick = 
            this.pingRenderer.horizontal_flip.bind(this.pingRenderer);

        // auto resizing of display area.
        window.onresize = this.match_display_size.bind(this);
        this.match_display_size();
    }

    async ping_callback(content)
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
            let data = new Uint8Array(await content.fetch_cached_data('data'));

            this.pingRenderer.set_ping_data(metadata, data.subarray(metadata.imageOffset));
        }
        finally {
            this.busy = false;
        }
        this.draw();
    }
};

