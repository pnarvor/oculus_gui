attribute vec2 aVertexPosition;

uniform vec2 uScalingFactor;
uniform vec2 uRotationVector;

void main()
{
    mat2 R = mat2(uRotationVector.x, uRotationVector.y,
                 -uRotationVector.y, uRotationVector.x);
    
    gl_Position = vec4((R*aVertexPosition)*uScalingFactor, 0.0, 1.0);
}
