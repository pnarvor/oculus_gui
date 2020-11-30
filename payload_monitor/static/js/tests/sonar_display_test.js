
class SonarDisplay extends Display
{
    constructor(canvas)
    {
        super(canvas);

        this.pingRenderer = new ImageRenderer(this.gl);
        this.add_renderer(this.pingRenderer);

        this.pingListener = new DataListener();
        this.pingListener.callbacks.push(this.ping_callback.bind(this));
    }

    async ping_callback(content)
    {
        let metadata = JSON.parse(content.content.metadata);
        let data = new Uint8Array(await content.fetch_cached_data());

        this.pingRenderer.set_image(new Shape(metadata.nBeams, metadata.nRanges),
                                    data.subarray(metadata.imageOffset));

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
