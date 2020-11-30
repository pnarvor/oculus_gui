
$(document).ready(function() {
    let canvas   = $("#main_display")[0];
    let display = new Display(canvas);
    canvas.naglContext = display;
    window.onresize = function() {
        $("#main_display")[0].naglContext
            .resize(window.innerWidth, window.innerHeight);
    };
    window.onresize();

    let renderer = new Renderer(display.gl);
    renderer.view.projection.set_at(0,0,0.5);
    renderer.view.projection.set_at(1,1,0.5);

    display.add_renderer(renderer);
     
    animate = function(currentTime) {
        $("#main_display")[0].naglContext.draw();
        window.requestAnimationFrame(animate);
    }
    animate();
});
