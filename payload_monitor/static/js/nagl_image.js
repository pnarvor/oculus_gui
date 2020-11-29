
create_image = function(shape) {
    let data = new Float32Array(4 * shape.size());
    let idx = 0;
    for(let h = 0; h < shape.height; h++) {
        for(let w = 0; w < shape.width; w++) {
            data[idx] = 0.0;
            data[idx + 1] = ((w+h)   & 0x1);
            data[idx + 2] = ((w+h+1) & 0x1);
            data[idx + 3] = 1.0;
            idx += 4;
        }
    }
    return data;
};



$(document).ready(function() {
    let canvas   = $("#main_display")[0];
    let display = new Display(canvas);
    canvas.naglContext = display;
    window.onresize = function() {
        $("#main_display")[0].naglContext
            .resize(window.innerWidth, window.innerHeight);
    };
    window.onresize();
    
    let view = new ImageView();
    let renderer = new Renderer(display.gl, view);
    
    let imageRenderer = new ImageRenderer(display.gl);
    let shape = new Shape(2,2);
    imageRenderer.set_rgb_image(shape, create_image(shape));

    display.add_renderer(renderer);
    display.add_renderer(imageRenderer);
     
    animate = function(currentTime) {
        $("#main_display")[0].naglContext.draw();
        window.requestAnimationFrame(animate);
    }
    animate();
});
