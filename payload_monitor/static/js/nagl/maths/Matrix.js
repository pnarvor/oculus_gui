

class Matrix
{
    static Linspace(nRows=undefined, nCols=undefined) {
        let res = Matrix.New(nRows, nCols);
        if(nRows == undefined) {
            throw Error("Matrix.constructor : you must give at least one dimension size.");
        }
        if(nCols == undefined) {
            nCols = nRows;
        }
        let data = new Float32Array(nRows*nCols);
        for(let i = 0; i < data.length; i++)
            data[i] = i;
        return new Matrix(data, nRows, nCols);
    }
    
    static Zeros(nRows, nCols) {
        if(nRows == undefined) {
            throw Error("Matrix.constructor : you must give at least one dimension size.");
        }
        if(nCols == undefined) {
            nCols = nRows;
        }
        let data = new Float32Array(nRows*nCols);
        for(let i in data) {
            data[i] = 0.0;
        }
        return new Matrix(data, nRows, nCols);
    }

    static Identity(nRows, nCols) {
        let res = Matrix.Zeros(nRows, nCols);
        for(let i = 0; i < Math.min(res.rows, res.cols); i++) {
            res.set_at(i,i,1.0);
        }
        return res;
    }

    static New(nRows=undefined, nCols=undefined) {
        if(nRows == undefined) {
            throw Error("Matrix.constructor : you must give at least one dimension size.");
        }
        if(nCols == undefined) {
            nCols = nRows;
        }
        return new Matrix(new Float32Array(nRows*nCols), nRows, nCols);
    }

    // this construct a view on the data
    constructor(array, nRows=undefined, nCols=undefined,
                rStride=undefined, cStride=undefined)
    {
        if(nRows == undefined) {
            throw Error("Matrix.constructor : you must give at least one dimension size.");
        }
        if(nCols == undefined) {
            nCols = nRows;
        }
        // defaults to row major ordering
        if(rStride == undefined) {
            rStride = 1;
        }
        if(cStride == undefined) {
            cStride = nRows;
        }
        // fix this (see #private properties)
        //Object.defineProperty(this, 'rows', {value : nRows,       writable : false});
        //Object.defineProperty(this, 'cols', {value : nCols,       writable : false});
        //Object.defineProperty(this, 'size', {value : nRows*nCols, writable : false});
        //Object.defineProperty(this, 'rStride', {value : rStride, writable : false});
        //Object.defineProperty(this, 'cStride', {value : cStride, writable : false});
        this.rows    = nRows;
        this.cols    = nCols;
        this.size    = nRows*nCols;
        this.rStride = rStride;
        this.cStride = cStride;
        this.elms = array;
    }
    print() {
        console.log("Matrix " + this.rows.toString() + "x" + this.cols.toString());
        for(let i = 0; i < this.rows; i++) {
            let line = this.at(i,0);
            for(let j = 1; j < this.cols; j++) {
                line += ", " + this.at(i,j).toString();
            }
            console.log(line);
        }
    }
    linear_index(row,col) {
        return this.rStride*row + this.cStride*col;
    }
    at(row, col) {
        return this.elms[this.linear_index(row,col)];
    }
    set_at(row, col, value) { this.elms[this.linear_index(row,col)] = value; }

    equal_shape(other)   { return other.rows == this.rows && other.cols == this.cols; }
    equal_shape_t(other) { return other.cols == this.rows && other.rows == this.cols; }
    equal_size(other)    { return other.size == this.size; }

    copy() { 
        return new Matrix(this.elms, this.rows, this.cols,
                          this.rStride, this.cStride);
    }
    copy_from(other) { 
        if(!this.equal_shape(other))
            throw Error("Matrix.copy_from : shape error");
        for(let i = 0; i < this.rows; i++) {
            for(let j = 0; j < this.cols; j++) {
                this.set_at(i,j,other.at(i,j));
            }
        }
    }
    deepcopy() {
        let res = Matrix.New(this.rows, this.cols);
        res.copy_from(this);
        return res;
    }
    
    transpose() {
        [this.rows, this.cols, this.rStride, this.cStride] = 
            [this.cols, this.rows, this.cStride, this.rStride];
        return this;
    }
    transposed() {
        let res = this.copy();
        res.transpose();
        return res;
    }

    col(idx) {
        return new Matrix(this.elms.subarray(this.cStride*idx),
                              this.rows, 1,
                              this.rStride, this.cStride);
    }
    row(idx) {
        return new Matrix(this.elms.subarray(this.rStride*idx),
                              1, this.cols,
                              this.rStride, this.cStride);
    }

    dot(other) {
        if(!this.equal_shape_t(other))
            throw Error("Matrix.dot : shape error");
        let res = 0.0;
        for(let i = 0; i < this.rows; i++) {
            for(let j = 0; j < this.cols; j++) {
                res += this.at(i,j)*other.at(j,i);
            }
        }
        return res;
    }

    multiply(other) {
        if(!this.equal_shape_t(other))
            throw Error("Matrix.multiply : shape error");
        let res = Matrix.New(this.rows, this.cols);
        for(let i = 0; i < res.rows; i++) {
            for(let j = 0; j < res.cols; j++) {
                res.set_at(i,j, this.row(i).dot(other.col(j)));
            }
        }
        return res;
    }

    force_column_major() {
        if(this.rStride == 1 && this.cStride == this.rows) {
            return this;
        }
        else {
            return this.deepcopy();
        }
    }

    is_square(size=undefined) {
        if(size == undefined) {
            return this.rows == this.cols;
        }
        else {
            return this.rows == size && this.cols == size;
        }
    }
};
