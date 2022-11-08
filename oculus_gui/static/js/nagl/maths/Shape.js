

class Shape
{
    constructor(width=undefined, height=undefined)
    {
        if(width == undefined || height == undefined) {
            throw Error("Shape() : width and height must be defined");
        }
        this.width  = width;
        this.height = height;
    }

    size() {
        return this.width*this.height;
    }

    ratio() {
        return this.width / this.height;
    }
};
