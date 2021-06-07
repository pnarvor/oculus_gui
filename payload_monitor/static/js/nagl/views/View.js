
class View
{
    constructor(projection=Matrix.Identity(4),
                screen = new Shape(1.0,1.0))
    {
        if(!projection.is_square(4)) {
            throw Error("View() : projection must be a square 4 matrix");
        }
        this.screen     = new Shape(screen.width, screen.height);
        this.projection = projection.force_column_major();
    }

    update_projection() {
        //to be reimplemented in subclasses.
    }

    set_screen_shape(shape) {
        this.screen = shape;
        this.update_projection();
    }
    
    // getters
    projection_matrix() {
        return this.projection;
    }

    full_matrix() {
        return this.projection;
    }
    
    get_screen_position(x, y = 0, z = 0, w = 1.0) {
        let view = this.full_matrix().force_column_major();
        let data = view.elms;
        
        // moving to clip space
        let sw = data[3]*x + data[7]*y + data[11]*z + data[15]*w;
        let sx = (data[0]*x + data[4]*y +  data[8]*z + data[12]*w) / sw;
        let sy = (data[1]*x + data[5]*y +  data[9]*z + data[13]*w) / sw;

        // to screen space (from top left)
        return [0.5*this.screen.width*(sx + 1.0),
                0.5*this.screen.height*(1.0 - sy)];
    }
};


