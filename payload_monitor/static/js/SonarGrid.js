class SonarGrid extends Renderer
{
    static vertexShader =
    `#version 300 es
    
    in float point;
    uniform mat4 view;

    uniform vec2  origin;
    uniform float iBeamOpening;
    uniform float widthScale;
    uniform float iRange;
    uniform vec4 cIn;

    out vec4 c;
    
    void main()
    {
        vec2 v0     = origin + iRange*vec2(widthScale*sin(iBeamOpening*point), 
                                           -cos(iBeamOpening*point));
        gl_Position = view*vec4(v0, 0.0, 1.0);
        c = cIn;
    }`;

    static sidesVertexShader =
    `#version 300 es
    
    in vec2 point;
    uniform mat4 view;

    uniform vec2  origin;
    uniform float iBeamOpening;
    uniform float widthScale;
    uniform vec4 cIn;

    out vec4 c;
    
    void main()
    {
        vec2 v0     = origin + point.y*vec2(widthScale*sin(iBeamOpening*point.x), 
                                            -cos(iBeamOpening*point.x));
        gl_Position = view*vec4(v0, 0.0, 1.0);
        c = cIn;
    }`;

    static fragmentShader =
    `#version 300 es
    #ifdef GL_ES
        precision highp float;
    #endif

    in  vec4 c; 
    out vec4 outColor;
    
    void main()
    {
        outColor = c;
    }`;

    constructor(gl, view, size = 512, 
                color = new Float32Array([0.6,0.6,0.7,1.0]))
    {
        super(gl, view,
              SonarGrid.vertexShader,
              SonarGrid.fragmentShader);
        this.color = color;
        this.size = size;

        let data  = new Float32Array(size);
        for(let i = 0; i < data.length; i++)
            data[i] = i / (data.length - 1) - 0.5;
        this.points = this.gl.createBuffer();
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.points);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, data, this.gl.STATIC_DRAW);
        
        // loading sides program
        this.sidesProgram = new Program(gl,
            [new Shader(gl, SonarGrid.sidesVertexShader,   gl.VERTEX_SHADER),
             new Shader(gl, SonarGrid.fragmentShader, gl.FRAGMENT_SHADER)]);
        data  = new Float32Array([-0.5,0.0,-0.5,2.0,0.5,0.0,0.5,2.0]);
        this.sides = this.gl.createBuffer();
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.sides);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, data, this.gl.STATIC_DRAW);
        
        this.range            = 0;
        this.beamOpening      = 0;
        this.ticks            = []
        this.targetTicksCount = 5;
    }
    
    update_ticks() {
        if(this.range == this.view.range && this.beamOpening == this.view.beamOpening) {
            return;
        }

        // new range. Clearing current ticks
        this.ticks = []
        let tickValues = Tick.nice_tick_values(0.0, this.view.range, 5);
        for(const v of tickValues) {
            if(v <= 0) continue;
            if(v > this.view.range) break;

            let tick = new Tick(v, this.view.get_tick_position(v));

            this.ticks.push(tick);
        }

        this.range       = this.view.range;
        this.beamOpening = this.view.beamOpening;
    }

    draw() {
        let view = this.view.full_matrix();

        this.draw_sides(view);
        for(const tick of this.ticks) {
            this.draw_range(view, tick.value / this.range, true);
        }
        this.draw_range(view, 1.0, false);

    }

    draw_sides(view) {

        this.sidesProgram.use();
        this.gl.uniformMatrix4fv(this.sidesProgram.getUniformLocation("view"), false,
            view.force_column_major().elms);
        this.gl.uniform2f(this.sidesProgram.getUniformLocation("origin"), 0.0, 1.0);
        this.gl.uniform1f(this.sidesProgram.getUniformLocation("iBeamOpening"),
                          this.view.beamOpening);
        this.gl.uniform1f(this.sidesProgram.getUniformLocation("widthScale"), 
                          0.5 / Math.sin(0.5*this.view.beamOpening));
        this.gl.uniform4fv(this.sidesProgram.getUniformLocation("cIn"), this.color);

        let loc = this.sidesProgram.getAttribLocation("point");
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.sides);
        this.gl.enableVertexAttribArray(loc);
        this.gl.vertexAttribPointer(loc, 2, this.gl.FLOAT, false, 0, 0);

        this.gl.drawArrays(this.gl.LINES, 0, 4);
    }

    draw_range(view, range, withTicks = false) {

        this.renderProgram.use();
        this.gl.uniformMatrix4fv(this.renderProgram.getUniformLocation("view"), false,
            view.force_column_major().elms);
        this.gl.uniform2f(this.renderProgram.getUniformLocation("origin"), 0.0, 1.0);
        if(withTicks) {
            this.gl.uniform1f(this.renderProgram.getUniformLocation("iBeamOpening"),
                              this.view.beamOpening + 0.02 / range);
        }
        else {
            this.gl.uniform1f(this.renderProgram.getUniformLocation("iBeamOpening"),
                              this.view.beamOpening);
        }
        this.gl.uniform1f(this.renderProgram.getUniformLocation("widthScale"), 
                          0.5 / Math.sin(0.5*this.view.beamOpening));
        this.gl.uniform1f(this.renderProgram.getUniformLocation("iRange"), 2.0*range);
        this.gl.uniform4fv(this.renderProgram.getUniformLocation("cIn"), this.color);

        let loc = this.renderProgram.getAttribLocation("point");
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.points);
        this.gl.enableVertexAttribArray(loc);
        this.gl.vertexAttribPointer(loc, 1, this.gl.FLOAT, false, 0, 0);

        this.gl.drawArrays(this.gl.LINE_STRIP, 0, this.size);
    }
};


