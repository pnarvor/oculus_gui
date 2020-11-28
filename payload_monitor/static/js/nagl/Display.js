

class Display
{
    constructor(canvas)
    {
        this.canvas = canvas;
        this.gl     = this.canvas.getContext("webgl");

        this.views     = []
        this.renderers = []
    }

    resize(width, height) {
        this.canvas.width  = width;
        this.canvas.height = height;
    }

    draw() {
        console.log("Drawing");
        for(view of this.views) {
            view.set_screen_size(this.canvas.width, this.canvas.height);
        }
        this.gl.viewport(0,0, this.canvas.width, this.canvas.height);
        this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.GL_DEPTH_BUFFER_BIT);
        for(renderer of this.renderers) {
            renderer.draw();
        }
    }
};
