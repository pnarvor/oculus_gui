
$(document).ready(function() {
    let canvas   = $("#main_display")[0];
    let renderer = new Display(canvas);
    canvas.renderer = renderer;

    let mat0 = Matrix.Linspace(4);
    mat0.print();
    mat0.col(1).print();
    mat0.row(1).print();

    mat0.transposed().print();
    mat0.print();
    mat0.transpose().print();
    mat0.print();

    console.log(mat0.col(1).dot(mat0.row(1)));
    mat0.multiply(mat0).print();
    
    animate = function(currentTime) {
        $("#main_display")[0].renderer.draw();
        window.requestAnimationFrame(animate);
    }
    //animate();
});
