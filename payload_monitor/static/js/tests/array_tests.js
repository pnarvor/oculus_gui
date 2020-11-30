
function window_resize()
{
    $('#status_div')[0].pingDisplay
        .resize(window.innerWidth, window.innerHeight);
}

function window_onclose()
{
    $('#status_div')[0].pingDisplay.free();
    return null;
}

class PingDisplay
{
    constructor(divContainer)
    {
        this.container = divContainer;


        THREE.Object3D.DefaultUp.set(0,0,1);
        this.scene    = new THREE.Scene();
        //this.camera   = new THREE.PerspectiveCamera(75, this.width / this.height);
        this.camera   = new THREE.Camera();
        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.container.append(this.renderer.domElement);

        this.resize(window.innerWidth, window.innerHeight);

        this.generate_image(64,64);
        
        this.load_scene();

        //this.update_canvas(width, height);
    }

    free()
    {
        //this.plane.geometry.dispose();
        //this.plane.material.dispose();
        //this.texture.dispose();
        //this.scene.dispose();
        //this.renderer.dispose();
    }

    resize(width, height) {
        this.width  = width;
        this.height = height;
        this.renderer.setSize(this.width, this.height);
    }

    load_scene() {

        let grid0 = new THREE.GridHelper(1, 100, 0x444444, 0x660000);
        grid0.rotation.x = Math.PI / 2.0;
        let grid1 = new THREE.GridHelper(1, 10);
        grid1.rotation.x = Math.PI / 2.0;

        
        //this.texture = new THREE.Texture(this.image);
        //this.texture.format = THREE.RGBAFormat;
        //this.texture.needsUpdate = true;
        
        let planeWidth = 1.5
        this.plane = new THREE.Mesh(new THREE.PlaneGeometry(planeWidth, planeWidth),
                                    new THREE.MeshBasicMaterial({map: this.texture}));
        console.log(this.plane);

        //this.scene.add(new THREE.AxesHelper(1.0));
        //this.scene.add(grid0);
        //this.scene.add(grid1);
        this.scene.add(this.plane);
    }

    render() {
        this.renderer.render(this.scene, this.camera);
    }

    //async generate_image(width, height) {
    generate_image(width, height) {
        let data = new Uint8ClampedArray(width*height*4);
        for(let h = 0; h < height; h++) {
            for(let w = 0; w < width; w++) {
                let value = 255*((w + h) & 0x1);
                //data[4*(width*h + w)]     = value;
                //data[4*(width*h + w) + 1] = value;
                //data[4*(width*h + w) + 2] = value;
                //data[4*(width*h + w) + 3] = 255;
                data[4*(width*h + w)]     = 0;
                data[4*(width*h + w) + 1] = 255*((w + h) & 0x1);
                data[4*(width*h + w) + 2] = 255*((w + h + 1) & 0x1);
                data[4*(width*h + w) + 3] = 255;

            }
        }
        this.image = new ImageData(data, width);
        this.texture = new THREE.DataTexture(data, width, height);
    }
    
    async update_canvas(width, height) {

        console.log("Update canvas");
        let img = await this.generate_image(width, height);
        console.log(img);
        
        let canvas  = $('#img0')[0];
        canvas.width  = width;
        canvas.height = height;

        let context = canvas.getContext('2d');

        context.drawImage(img, 0, 0);
    }
};

$(document).ready(function() {
    $('#status_div')[0].pingDisplay = new PingDisplay($('#status_div')[0]);
    window.onresize       = window_resize;
    window.onbeforeunload = window_onclose;
    animate = async function() {
        await new Promise(r => setTimeout(r, 100));
        $('#status_div')[0].pingDisplay.render();
        requestAnimationFrame(animate);
    }
    animate();
});
