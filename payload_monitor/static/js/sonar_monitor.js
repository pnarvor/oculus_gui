

$(document).ready(function() {

    let display = new SonarDisplay($(".sonar-gui")[0]);
    
    let sonarControl    = new ReconfigureGUI($("#reconf_container")[0], "oculus_sonar");
    
    let recorderControl = new RecorderGUI($("#recorder-gui")[0], "rosbag_recorder");
});



