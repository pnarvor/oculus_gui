
class View
{
    constructor(projection=Matrix.Identity(4),
                screen = {width : 1.0, height : 1.0})
    {
        if(!projection.is_square(4)) {
            throw Error("View() : projection must be a square 4 matrix");
        }
        this.screen     = screen;
        this.projection = projection.force_column_major();
    }

    update_projection() {
        //to be reimplemented in subclasses.
    }

    set_screen_shape(screen) {
        if(typeof(screen.width)  == "undefined" ||
           typeof(screen.height) == "undefined") {
            throw Error("View.set_screen_shape : screen parameter " +
                        "must have both a width and a height argument");
        }
        this.screen = screen;
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
