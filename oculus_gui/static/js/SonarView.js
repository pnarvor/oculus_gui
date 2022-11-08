

class SonarView extends ImageView
{
    constructor(beamOpening = 1.5, range = 1.0) {
        super();
        this.flipViewMatrix = Matrix.Identity(4);
        this.set_beam_opening(beamOpening);
        this.set_range(range);
    }

    set_beam_opening(beamOpening) {
        this.beamOpening = beamOpening;
        this.set_image_shape(new Shape(2.0*Math.sin(0.5*this.beamOpening), 1.0));
    }

    set_range(range) {
        this.range = range;
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

