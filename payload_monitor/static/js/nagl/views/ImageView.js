

class ImageView extends View {
    constructor(imageShape = new Shape(1.0, 1.0)) {
        super();
        this.imageShape = imageShape;
    }

    update_projection() {
        this.projection = Matrix.Identity(4);
        let metaRatio = this.screen.ratio() / this.imageShape.ratio();
        if(metaRatio > 1.0) {
            this.projection.set_at(0,0, 1.0 / metaRatio);
        }
        else {
            this.projection.set_at(1,1, metaRatio);
        }
    }
};
