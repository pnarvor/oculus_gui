

$(document).ready(function() {
    let displayDiv = $(".sonar-gui")[0];
    console.log(displayDiv);
    
    displayDiv.dataFetcher = new DataListener('/ws/sonar_data/');
});
