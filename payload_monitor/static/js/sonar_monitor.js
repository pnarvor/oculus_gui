
$(document).ready(function() {
    let display = new SonarDisplay($("#main_display")[0]);


   let sideNav = document.createElement("ul");
   sideNav.id = "slide-out";
   sideNav.classList.add("sidenav", "sidenav-fixed");
   $("#reconf_container")[0].appendChild(sideNav);
   $('.sidenav').sidenav({edge:'right'});

    let control = new ReconfigureGUI(sideNav);
    //window.onresize = function() {
    //    $("#main_display")[0].naglContext
    //        .resize(window.innerWidth, window.innerHeight);
    //};
    //window.onresize();

});
