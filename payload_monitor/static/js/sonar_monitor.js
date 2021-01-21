

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

$(document).ready(function() {

    let canvas = document.createElement("canvas");
    canvas.setAttribute("id", "display");
    $("#main")[0].appendChild(canvas);
    let display = new SonarDisplay(canvas);
    
    window.onresize = update_size;
    window.onresize();
    
    let sonarControl    = new ReconfigureGUI($("#reconf_container")[0], "oculus_sonar");
    
    let recorderControl = new RecorderGUI($("#recorder-gui")[0], "rosbag_recorder");
});



