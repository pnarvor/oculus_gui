
function check_gl_error(gl)
{
    let err = gl.getError();
    if(err == gl.NO_ERROR) return;
    
    let log;
    switch(err)
    {
        case gl.NO_ERROR:                      log = "Got gl.NO_ERROR"; break;
        case gl.INVALID_ENUM:                  log = "Got gl.INVALID_ENUM"; break;
        case gl.INVALID_VALUE:                 log = "Got gl.INVALID_VALUE"; break;
        case gl.INVALID_OPERATION:             log = "Got gl.INVALID_OPERATION"; break;
        case gl.INVALID_FRAMEBUFFER_OPERATION: log = "Got gl.INVALID_FRAMEBUFFER_OPERATION"; break;
        case gl.OUT_OF_MEMORY:                 log = "Got gl.OUT_OF_MEMORY"; break;
        case gl.CONTEXT_LOST_WEBGL:            log = "Got gl.CONTEXT_LOST_WEBGL"; break;
        default:                               log = "Got unknown error"; break;
    }

    throw new Error(log);
}

class Shader
{
    constructor(gl, source, type)
    {
        //this.source = source;
        this.type   = type;
        this.shader = gl.createShader(type);

        gl.shaderSource(this.shader, source);
        gl.compileShader(this.shader);
    
        if (!gl.getShaderParameter(this.shader, gl.COMPILE_STATUS)) {
            console.error("Error compiling shader\n"
                          + source + "\n"
                          + gl.getShaderInfoLog(this.shader));
        }
    }
};

class Program
{
    constructor(gl, shaders)
    {
        this.gl = gl;
        this.program = gl.createProgram();

        for(const shader of shaders) {
            gl.attachShader(this.program, shader.shader);
        }
        gl.linkProgram(this.program);

        if (!gl.getProgramParameter(this.program, gl.LINK_STATUS)) {
            console.error("Error linking shader program: "
                          + gl.getProgramInfoLog(this.program));
        }
    }

    use() {
        this.gl.useProgram(this.program);
    }

    getUniformLocation(name) {
        let loc = this.gl.getUniformLocation(this.program, name);
        if(loc == null) {
            throw new Error("Cannot get uniform location for name " + name);
        }
        return loc;
    }

    getAttribLocation(name) {
        let loc = this.gl.getAttribLocation(this.program, name);
        if(loc == null) {
            throw new Error("Cannot get uniform location for name " + name);
        }
        return loc;
    }
};
