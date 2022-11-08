

class Renderer
{
    static defaultVertexShader =
    `#version 300 es
    in vec3 point;
    in vec3 color;
    uniform mat4 view;
    out vec3 c;
    
    void main()
    {
        gl_Position = view*vec4(point, 1.0f);
        //gl_Position = vec4(point, 1.0f);
        c = color;
    }`;
    static defaultFragmentShader =
    `#version 300 es
    #ifdef GL_ES
        precision highp float;
    #endif
    
    in vec3 c;
    out vec4 outColor;
    
    void main()
    {
        outColor = vec4(c, 1.0f);
    }`;

    constructor(gl, view = new View(), 
                vertexShader = Renderer.defaultVertexShader,
                fragmentShader = Renderer.defaultFragmentShader) {
        this.gl = gl;
        this.renderProgram = new Program(gl,
            [new Shader(gl, vertexShader,   gl.VERTEX_SHADER),
             new Shader(gl, fragmentShader, gl.FRAGMENT_SHADER)]);
        this.view = view;
    }

    set_view(view) {
        this.view = view;
    }

    draw() {
        let pointsData = new Float32Array([0,0,0,
                                           1,0,0,
                                           0,0,0,
                                           0,1,0,
                                           0,0,0,
                                           0,0,1]);
        let colorsData = new Float32Array([1,0,0,
                                           1,0,0,
                                           0,1,0,
                                           0,1,0,
                                           0,0,1,
                                           0,0,1]);
        let points = this.gl.createBuffer();
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, points);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, pointsData, this.gl.STATIC_DRAW);

        let colors = this.gl.createBuffer();
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, colors);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, colorsData, this.gl.STATIC_DRAW);
        
        //this.gl.lineWidth(1);

        this.renderProgram.use();

        this.gl.uniformMatrix4fv(this.renderProgram.getUniformLocation("view"),
                                 false, this.view.full_matrix().force_column_major().elms);

        let loc = this.renderProgram.getAttribLocation("point");
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, points);
        this.gl.enableVertexAttribArray(loc);
        this.gl.vertexAttribPointer(loc, 3, this.gl.FLOAT, false, 0, 0);

        loc = this.renderProgram.getAttribLocation("color");
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, colors);
        this.gl.enableVertexAttribArray(loc);
        this.gl.vertexAttribPointer(loc, 3, this.gl.FLOAT, false, 0, 0);

        this.gl.drawArrays(this.gl.LINES, 0, 6);
    }
};
