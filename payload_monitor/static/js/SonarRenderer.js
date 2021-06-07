

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
    uniform vec4 zeroColor;

    uniform vec2  origin;
    uniform float iBeamOpening;
    uniform float widthScale;

    uniform vec2  gainScaling;
    
    out vec4 outColor;
    
    void main()
    {
        // local coordinates inside the image (bounded by [-1,1])
        vec2 v0 = vec2(-0.5f*(uv.y - origin.y),
                       (uv.x - origin.x) * widthScale);
        // coordinates inside the sonar fan (angle, range).
        vec2 v = vec2(atan(v0.y, v0.x) * iBeamOpening + 0.5f,
                      length(v0));

        v.x = gainScaling.x * v.x + gainScaling.y;
        if(v.x >= gainScaling.y && v.x <= 1.0f && v.y <= 1.0f)
        //if(v.x >= 0.0f && v.x <= 1.0f && v.y <= 1.0f)
            outColor = texture(colormap, vec2(texture(tex, v).x, 0.0f));
        else
            outColor = zeroColor;

    }`;

    constructor(gl) {
        super(gl, new SonarView(),
              SonarRenderer.vertexShader,
              SonarRenderer.fragmentShader);

        // cannot pre-allocate data without giving a full buffer
        this.texture  = this.gl.createTexture();
        this.gainSent = false;

        this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_S, this.gl.CLAMP_TO_EDGE);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_T, this.gl.CLAMP_TO_EDGE);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.NEAREST);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MAG_FILTER, this.gl.NEAREST);

        this.colormap = new Colormap(this.gl, Colormap.Viridis());
        //this.colormap = new Colormap(this.gl, Colormap.Gray());
        this.zeroColor = this.colormap.minColor;
    }

    horizontal_flip() {
        this.view.horizontal_flip();
    }

    vertical_flip() {
        this.view.vertical_flip();
    }

    set_ping_data(metadata, data) {
        if(metadata.fireMessage.masterMode == 1) {
            this.view.set_beam_opening(130.0 * Math.PI / 180.0);
        }
        else {
            this.view.set_beam_opening(80.0 * Math.PI / 180.0);
        }
        this.view.set_range(metadata.fireMessage.range);
        
        this.gainSent = (metadata.fireMessage.flags & 0x4) != 0;
        this.nBeams = metadata.nBeams;
        // gain is interleaved with data
        if(!this.gainSent) {
            this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
            this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.R8,
                               metadata.nBeams, metadata.nRanges, 0,
                               this.gl.RED, this.gl.UNSIGNED_BYTE, data);
        }
        else {
            this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
            this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.R8,
                               metadata.nBeams + 4, metadata.nRanges, 0,
                               this.gl.RED, this.gl.UNSIGNED_BYTE, data);
        }
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
        
        this.gl.uniformMatrix4fv(this.renderProgram.getUniformLocation("view"), false,
            this.view.full_matrix().force_column_major().elms);
        
        this.gl.uniform1f(this.renderProgram.getUniformLocation("iBeamOpening"),
                          1.0 / this.view.beamOpening);
        this.gl.uniform1f(this.renderProgram.getUniformLocation("widthScale"),
                          Math.sin(0.5*this.view.beamOpening));
        this.gl.uniform2f(this.renderProgram.getUniformLocation("origin"), 0.0, 1.0);
        this.gl.uniform4fv(this.renderProgram.getUniformLocation("zeroColor"), this.zeroColor);
        
        // this part is to shift the texture a bit to avoid display of range gains.
        let alpha = 1.0;
        let beta  = 0.0;
        if(this.gainSent) {
            alpha = (this.nBeams - 4) / this.nBeams;
            beta  = 4 / this.nBeams;
        }
        this.gl.uniform2f(this.renderProgram.getUniformLocation("gainScaling"), alpha, beta);

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
