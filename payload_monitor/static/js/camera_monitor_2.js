

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
    // $("#display1")[0].naglContext.resize(innerWidth/2, innerHeight);
    // $("#display2")[0].naglContext.resize(innerWidth/2, innerHeight);
}

$(document).ready(function() {

    let imageDiv1 = document.createElement("img");
    imageDiv1.style.width  = '50%';
    imageDiv1.style.height = 'auto';
    $("#main")[0].appendChild(imageDiv1);
    let display1 = new CameraDisplay2(imageDiv1,
                                      //"/camera0/image_raw/compressed",
                                      //"/stereo/left/image_raw/compressed",
                                      "/stereo/left/image_raw/munu_compressed/compressed",
                                      "sensor_msgs/CompressedImage");
    
    let imageDiv2 = document.createElement("img");
    imageDiv2.style.width = '50%';
    imageDiv2.style.height = 'auto';
    $("#main")[0].appendChild(imageDiv2);
    let display2 = new CameraDisplay2(imageDiv2,
                                      //"/camera1/image_raw/compressed",
                                      //"/stereo/right/image_raw/compressed",
                                      "/stereo/right/image_raw/munu_compressed/compressed",
                                      "sensor_msgs/CompressedImage");
    

    // let canvas1 = document.createElement("canvas");
    // //canvas1.width = 800;
    // //canvas1.height = 600;
    // canvas1.setAttribute("id", "display1");
    // $("#main")[0].appendChild(canvas1);
    // let display1 = new CameraDisplay(canvas1,'/camera0/image_raw');

    // let canvas2 = document.createElement("canvas");
    // //canvas.width = 800;
    // //canvas.height = 600;
    // canvas2.setAttribute("id", "display2");
    // $("#main")[0].appendChild(canvas2);
    // let display2 = new CameraDisplay(canvas2,'/camera1/image_raw');
    // 
    // window.onresize = update_size;
    // window.onresize();

    // //let imageListener = new DataListener('/ws/subscribe/camera_image_raw/');
    // //imageListener.callbacks.push(callback);
    // //$("#main")[0].appendChild(imageListener);
    
    let cameraControl   = new ReconfigureGUI($("#reconf_container")[0], "ueye_cam")
    let recorderControl = new RecorderGUI($("#recorder-gui")[0], "rosbag_recorder");
});
