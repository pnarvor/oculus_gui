

$(document).ready(function() {

    let map = L.map('map').setView({lon: 0, lat: 0}, 2);
    $("#map")[0].map = map;
    
    // add the OpenStreetMap tiles
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; <a href="https://openstreetmap.org/copyright">OpenStreetMap contributors</a>'
    }).addTo(map);
    
    // show the scale bar on the lower left corner
    L.control.scale({imperial: true, metric: true}).addTo(map);

    let marker = L.marker([37.780012, -122.404827]).addTo(map);
    let polylinePoints = [
        [37.781814, -122.404740],
        [37.781719, -122.404637],
        [37.781489, -122.404949],
        [37.780704, -122.403945],
        [37.780012, -122.404827]
    ];
    let line = L.polyline(polylinePoints).addTo(map);
    map.setView([37.780012, -122.404827], 18);

    console.log(line);
});
