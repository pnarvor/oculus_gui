class Display
{
    constructor(canvas)
    {
        canvas.naglContext = this; // canvas will hold a reference to this.

        this.canvas = canvas;
        this.gl     = this.canvas.getContext("webgl2", {antialias : true});

        this.gl.clearColor(0.0,0.0,0.0,1.0);

        this.views     = []
        this.renderers = []
    }

    resize(width, height) {
        this.canvas.width  = width;
        this.canvas.height = height;
        for(let view of this.views) {
            view.set_screen_shape(this.screen_shape());
        }
    }

    match_display_size() {
        this.resize(this.canvas.clientWidth,
                    this.canvas.clientHeight);
    }

    screen_shape() {
        return new Shape(this.canvas.width, this.canvas.height);
    }

    add_view(view) {
        for(const v of this.views) {
            if(Object.is(v, view)) {
                return;
            }
        }
        this.views.push(view);
    }

    add_renderer(renderer) {
        this.add_view(renderer.view);
        this.renderers.push(renderer);
    }

    draw() {
        this.gl.viewport(0, 0, this.canvas.width, this.canvas.height);
        this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.GL_DEPTH_BUFFER_BIT);
        for(let renderer of this.renderers) {
            renderer.draw();
        }
    }
};
