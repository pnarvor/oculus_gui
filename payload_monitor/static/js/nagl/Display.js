

class Display
{
    constructor(canvas)
    {
        this.canvas = canvas;
        this.gl     = this.canvas.getContext("webgl2", {antialias : false});

        this.gl.clearColor(0.0,0.0,0.0,1.0);

        this.views     = []
        this.renderers = []
    }

    resize(width, height) {
        this.canvas.width  = width;
        this.canvas.height = height;
    }

    screen_shape() {
        return {width  : this.canvas.width,
                height : this.canvas.height};
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
        for(let view of this.views) {
            view.set_screen_shape(this.screen_shape());
        }
        this.gl.viewport(0,0, this.canvas.width, this.canvas.height);
        this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.GL_DEPTH_BUFFER_BIT);
        for(let renderer of this.renderers) {
            renderer.draw();
        }
    }
};
