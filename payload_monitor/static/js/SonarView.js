

class SonarView extends ImageView
{
    constructor(imageShape = new Shape(1.0, 1.0)) {
        super(imageShape);
        this.flipViewMatrix = Matrix.Identity(4);
    }

    horizontal_flip() {
        this.flipViewMatrix.set_at(0,0, -this.flipViewMatrix.at(0,0));
    }

    vertical_flip() {
        this.flipViewMatrix.set_at(1,1, -this.flipViewMatrix.at(1,1));
    }

    full_matrix() {
        return this.flipViewMatrix.multiply(ImageView.prototype.full_matrix.call(this));
    }
};

