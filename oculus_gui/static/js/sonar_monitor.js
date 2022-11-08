

$(document).ready(function() {

    let display = new SonarDisplay($(".sonar-gui")[0]);

    let sonarControl = new ReconfigureGUI($("#reconf_container")[0],
                                            "/ws/reconfigure_client/");
});


