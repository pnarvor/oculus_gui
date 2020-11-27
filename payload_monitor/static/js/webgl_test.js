
function window_resize()
{
    $('#main_display')[0].renderer
        .resize(window.innerWidth, window.innerHeight);
}

class WebGL_Renderer
{
    constructor(canvas)
    {
        this.canvas  = canvas;
        this.gl      = this.canvas.getContext("webgl");
        
        this.init();

    }

    async init()
    {

        let vertexSource   = await $.get($("#vertex_shader").attr("src"));
        let fragmentSource = await $.get($("#fragment_shader").attr("src"));

        this.program = new Program(this.gl,
            [new Shader(this.gl, vertexSource,   this.gl.VERTEX_SHADER),
             new Shader(this.gl, fragmentSource, this.gl.FRAGMENT_SHADER)]);

        let points = new Float32Array([-0.5, 0.5, 0.5, 0.5, 0.5, -0.5,
                                       -0.5, 0.5, 0.5, -0.5, -0.5, -0.5]);

        this.points = this.gl.createBuffer();
        check_gl_error(this.gl);
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.points);
        check_gl_error(this.gl);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, points, this.gl.STATIC_DRAW);
        check_gl_error(this.gl);

        this.gl.clearColor(0.8,0.9,1.0,1.0);
        check_gl_error(this.gl);

        this.resize(window.innerWidth, window.innerHeight);

        check_gl_error(this.gl);

        this.currentAngle = 0.0;
        this.previousTime = 0.0;

        this.render();
    }

    render(currentTime=0.0)
    {
        let deltaAngle = 0.25*(currentTime - this.previousTime) / 1000.0;
        this.previousTime = currentTime;
        this.currentAngle = this.currentAngle + deltaAngle;
        while(this.currentAngle > 2.0*Math.PI)
            this.currentAngle -= 2.0*Math.PI;

        this.gl.viewport(0,0,this.canvas.width,this.canvas.height);
        this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.GL_DEPTH_BUFFER_BIT);

        this.program.use();
        
        this.gl.uniform2fv(this.program.getUniformLocation("uScalingFactor"), 
                           [1.0, this.canvas.width/ this.canvas.height]);
        this.gl.uniform2fv(this.program.getUniformLocation("uRotationVector"),
                           [Math.cos(this.currentAngle), Math.sin(this.currentAngle)]);
        this.gl.uniform4fv(this.program.getUniformLocation("uGlobalColor"),
                           [0.1,0.7,0.2,1.0]);

        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.points);
        let vertexLocation = this.program.getAttribLocation("aVertexPosition");
        this.gl.enableVertexAttribArray(vertexLocation);
        this.gl.vertexAttribPointer(vertexLocation, 2, this.gl.FLOAT, false, 0, 0);

        this.gl.drawArrays(this.gl.TRIANGLES, 0, 6);

        check_gl_error(this.gl);
        
        window.requestAnimationFrame(function(currentTime) {
            let renderer = $("#main_display")[0].renderer;

            renderer.render(currentTime);
        });
    }

    resize(width, height) {
        this.canvas.width  = width;
        this.canvas.height = height;

        //this.clear();
    }
    
    clear() {
        let context = this.canvas.getContext("2d");
        context.rect(0,0,this.canvas.width, this.canvas.height);
        context.fillStyle = "red";
        context.fill();
    }
};

$(document).ready(function() {

    $('#main_display')[0].renderer = new WebGL_Renderer($('#main_display')[0]);

    window.onresize       = window_resize;

    animate = async function() {
        //requestAnimationFrame(animate);
    }
    animate();
});
