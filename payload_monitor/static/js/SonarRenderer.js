

class SonarRenderer extends Renderer
{
    static vertexShader =
    `#version 300 es
    
    in vec2 point;
    out vec2 uv;
    uniform mat4 view;
    
    void main()
    {
        gl_Position = view*vec4(point, 0.0, 1.0);
        uv = point;
    }`;

    static fragmentShader =
    `#version 300 es
    #ifdef GL_ES
    	precision highp float;
    #endif
    
    in vec2 uv;
    uniform sampler2D tex;
    uniform sampler2D colormap;

    uniform vec2  origin;
    uniform float iBeamOpening;
    uniform float widthScale;
    
    out vec4 outColor;
    
    void main()
    {
        vec2 v0 = vec2(-0.5f*(uv.y - origin.y),
                       (uv.x - origin.x) * widthScale);
        vec2 v = vec2(atan(v0.y, v0.x) * iBeamOpening + 0.5f,
                      length(v0));
        if(v.x >= 0.0f && v.x <= 1.0f && v.y <= 1.0f)
            outColor = texture(colormap, vec2(texture(tex, v).x, 0.0f));
        else
            outColor = texture(colormap, vec2(0.0f, 0.0f));

    }`;

    constructor(gl) {
        super(gl, new ImageView(),
              //ImageRenderer.defaultVertexShader,
              //ImageRenderer.colormapFragmentShader);
              SonarRenderer.vertexShader,
              SonarRenderer.fragmentShader);

        // cannot pre-allocate data without giving a full buffer
        this.texture  = this.gl.createTexture();

        this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_S, this.gl.CLAMP_TO_EDGE);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_T, this.gl.CLAMP_TO_EDGE);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.NEAREST);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MAG_FILTER, this.gl.NEAREST);

        this.colormap = new Colormap(this.gl, Colormap.Viridis());

        this.beamOpening = 130.0 * Math.PI / 180.0;
        //this.beamOpening =  60.0 * Math.PI / 180.0;
    }

    set_ping_data(metadata, data) {
        
        if(metadata.fireMessage.masterMode == 1) {
            this.beamOpening = 130.0 * Math.PI / 180.0;
            //this.beamOpening = 90.0 * Math.PI / 180.0;
        }
        else {
            this.beamOpening =  60.0 * Math.PI / 180.0;
            //this.beamOpening =  30.0 * Math.PI / 180.0;
        }
        this.view.set_image_shape(new Shape(2.0*Math.sin(0.5*this.beamOpening), 1.0));

        this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
        this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.R8,
                           metadata.nBeams, metadata.nRanges, 0,
                           this.gl.RED, this.gl.UNSIGNED_BYTE, data);
    }

    draw() {
        let pointsData = new Float32Array([-1.0,-1.0,
                                            1.0,-1.0,
                                            1.0, 1.0,
                                           -1.0,-1.0,
                                            1.0, 1.0,
                                           -1.0, 1.0]);
        let points = this.gl.createBuffer();
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, points);
        this.gl.bufferData(this.gl.ARRAY_BUFFER, pointsData, this.gl.STATIC_DRAW);

        this.renderProgram.use();

        this.gl.uniformMatrix4fv(this.renderProgram.getUniformLocation("view"),
                                 false, this.view.full_matrix().force_column_major().elms);
        
        this.gl.uniform1f(this.renderProgram.getUniformLocation("iBeamOpening"),
                          1.0 / this.beamOpening);
        this.gl.uniform1f(this.renderProgram.getUniformLocation("widthScale"),
                          Math.sin(0.5*this.beamOpening));
        this.gl.uniform2f(this.renderProgram.getUniformLocation("origin"), 0.0, 1.0);

        let loc = this.renderProgram.getAttribLocation("point");
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, points);
        this.gl.enableVertexAttribArray(loc);
        this.gl.vertexAttribPointer(loc, 2, this.gl.FLOAT, false, 0, 0);

        this.gl.uniform1i(this.renderProgram.getUniformLocation("tex"), 0);
        this.gl.activeTexture(this.gl.TEXTURE0);
        this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
        
        this.gl.uniform1i(this.renderProgram.getUniformLocation("colormap"), 1);
        this.gl.activeTexture(this.gl.TEXTURE1);
        this.colormap.bind(this.gl.TEXTURE_2D);

        this.gl.drawArrays(this.gl.TRIANGLES, 0, 6);
    }
};
