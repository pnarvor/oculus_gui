

class ImageRenderer extends Renderer
{
    static defaultVertexShader =
    `#version 300 es
    
    in vec2 point;
    out vec2 uv;
    uniform mat4 view;
    
    void main()
    {
        gl_Position = view*vec4(point, 0.0, 1.0);
        uv.x = 0.5f*(point.x + 1.0f);
        uv.y = 0.5f*(1.0f - point.y);
    }`;
    
    static defaultFragmentShader =
    `#version 300 es
    #ifdef GL_ES
    	precision highp float;
    #endif
    
    in vec2 uv;
    uniform sampler2D tex;
    
    out vec4 outColor;
    
    void main()
    {
        outColor = texture(tex, uv);
        //outColor = vec4(0.0, uv.x, uv.y, 1.0);
    }`;

    constructor(gl) {
        super(gl, new ImageView(),
              ImageRenderer.defaultVertexShader,
              ImageRenderer.defaultFragmentShader);
        this.gl = gl;

        // cannot pre-allocate data without giving a full buffer
        this.texture = this.gl.createTexture();
    }

    set_rgb_image(shape, data) {
        this.view.set_image_shape(shape);
        
        let dataType;
        if(data instanceof Float32Array) {
            dataType = this.gl.FLOAT;
        }
        else {
            throw Error("Data format not supported");
        }

        this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
        this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.RGBA32F, 
                           shape.width, shape.height, 0,
                           //this.gl.RGB, dataType, data);
                           this.gl.RGBA, dataType, data);
        // The conversion from this.gl.RGB to this.gl.RGBA generates a warning.
        // However, with the C API counterpart, this is well defined (missing
        // color channels are set to 0, or 1 if it is the alpha channel). So
        // this should work ? (check it->does not...).
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_S, this.gl.CLAMP_TO_EDGE);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_T, this.gl.CLAMP_TO_EDGE);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.NEAREST);
        this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MAG_FILTER, this.gl.NEAREST);
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

        let loc = this.renderProgram.getAttribLocation("point");
        this.gl.bindBuffer(this.gl.ARRAY_BUFFER, points);
        this.gl.enableVertexAttribArray(loc);
        this.gl.vertexAttribPointer(loc, 2, this.gl.FLOAT, false, 0, 0);

        this.gl.uniform1i(this.renderProgram.getUniformLocation("tex"), 0);
        this.gl.activeTexture(this.gl.TEXTURE0);
        this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);

        this.gl.drawArrays(this.gl.TRIANGLES, 0, 6);
    }
};
