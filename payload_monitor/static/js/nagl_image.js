
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

    display.add_renderer(renderer);
     
    animate = function(currentTime) {
        $("#main_display")[0].naglContext.draw();
        window.requestAnimationFrame(animate);
    }
    animate();
});
