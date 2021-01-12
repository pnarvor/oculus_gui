

update_size = function() {
    let main = $("#main")[0];
    let sidenav = $(".reconf-sidenav")[0];
    if(sidenav !== undefined) {
        if(sidenav.M_Sidenav.isOpen) {
            main.style.marginRight = $(".sidenav").css("width");
        }
        else {
            main.style.marginRight = 0;
        }
    }
    else {
        main.style.marginRight = 0;
    }
    let innerWidth  = window.innerWidth  - parseInt(main.style.marginRight, 10);
    let innerHeight = window.innerHeight - parseInt($("#top-nav").css("height"), 10);
    $("#display")[0].naglContext.resize(innerWidth, innerHeight);
}

callback = function(msg) {
    console.log("Got image");
}

$(document).ready(function() {

    let canvas = document.createElement("canvas");
    canvas.width = 800;
    canvas.height = 600;
    canvas.setAttribute("id", "display");
    $("#main")[0].appendChild(canvas);
    let display = new CameraDisplay(canvas);
    
    //window.onresize = update_size;
    //window.onresize();

    //let imageListener = new DataListener('/ws/subscribe/camera_image_raw/');
    //imageListener.callbacks.push(callback);
    //$("#main")[0].appendChild(imageListener);
    
    let cameraControl   = new ReconfigureGUI($("#reconf_container")[0], "ueye_cam")
});
