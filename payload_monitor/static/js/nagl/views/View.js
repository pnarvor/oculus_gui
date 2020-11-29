
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
};
