class SonarDisplay extends Display
{
    constructor(container)
    {
        super(container.getElementsByClassName("sonar-canvas")[0]);

        this.container = container

        this.pingRenderer = new SonarRenderer(this.gl);
        this.sonarGrid = new SonarGrid(
            this.container.getElementsByClassName("sonar-display-col")[0],
            this.gl, this.pingRenderer.view);
        this.add_renderer(this.pingRenderer);
        this.add_renderer(this.sonarGrid);

        this.gl.clearColor(this.pingRenderer.zeroColor[0],
                           this.pingRenderer.zeroColor[1],
                           this.pingRenderer.zeroColor[2],
                           this.pingRenderer.zeroColor[3]);

        this.pingListener = new DataListener('/ws/sonar_data/');
        this.pingListener.callbacks.push(this.ping_callback.bind(this));

        this.busy = false;
        
        // Binding buttons to SonarDisplay control methods
        this.vFlipButton = container.getElementsByClassName("sonar-vflip")[0];
        this.vFlipButton.onclick = this.vertical_flip.bind(this);
        this.hFlipButton = container.getElementsByClassName("sonar-hflip")[0];
        this.hFlipButton.onclick = this.horizontal_flip.bind(this);

        // auto resizing of display area.
        window.onresize = this.match_display_size.bind(this);
        this.match_display_size();
    }

    vertical_flip() {
        this.pingRenderer.view.vertical_flip();
        this.sonarGrid.update_tick_positions();
    }

    horizontal_flip() {
        this.pingRenderer.view.horizontal_flip();
        this.sonarGrid.update_tick_positions();
    }

    match_display_size() {
        Display.prototype.match_display_size.call(this);
        this.sonarGrid.update_tick_positions();
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

            this.pingRenderer.set_ping_data(metadata, data.subarray(metadata.pingDataOffset));
            this.sonarGrid.beam_changed();
        }
        finally {
            this.busy = false;
        }
        this.draw();
    }
};

