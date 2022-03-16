

class GpsDisplay
{
    constructor(mapDiv, topicNames)
    {
        this.mapDiv = mapDiv;

        this.mapDiv.gpsDisplay = this;

        this.build_map();
        

        // this.msgSubscriber = new RosTopicListener(topicName, 'hemisphere_v500/StampedString');
        // this.msgSubscriber.callbacks.push(this.gps_callback.bind(this, topicName));
        
        this.subscribers = []
        for(let topicName of topicNames) {
            let subscriber = new RosTopicListener(topicName, 'hemisphere_v500/StampedString');
            subscriber.callbacks.push(this.gps_callback.bind(this));
            this.subscribers.push(subscriber);
        }
    }

    async gps_callback(content)
    {
        console.log(content.content.topic);
        let data = JSON.parse(content.content.scalars);
        console.log(data);
    }

    build_map(firstViewTarget = {lon: 0, lat: 0})
    {
        if(typeof this.map !== 'undefined') {
            return;
        }

        this.map = L.map('map').setView(firstViewTarget, 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
          maxZoom: 19,
          attribution: '&copy; <a href="https://openstreetmap.org/copyright">OpenStreetMap contributors</a>'
        }).addTo(this.map);

        // show the scale bar on the lower left corner
        L.control.scale({imperial: true, metric: true}).addTo(this.map);
    }
};


$(document).ready(function() {

    let gpsDisplay = new GpsDisplay($("#map")[0], ["/nmea"]);

    // let map = L.map('map').setView({lon: 0, lat: 0}, 2);
    // $("#map")[0].map = map;
    // 
    // // add the OpenStreetMap tiles
    // L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    //   maxZoom: 19,
    //   attribution: '&copy; <a href="https://openstreetmap.org/copyright">OpenStreetMap contributors</a>'
    // }).addTo(map);
    // 
    // // show the scale bar on the lower left corner
    // L.control.scale({imperial: true, metric: true}).addTo(map);

    // let marker = L.marker([37.780012, -122.404827]).addTo(map);
    // let polylinePoints = [
    //     [37.781814, -122.404740],
    //     [37.781719, -122.404637],
    //     [37.781489, -122.404949],
    //     [37.780704, -122.403945],
    //     [37.780012, -122.404827]
    // ];
    // let line = L.polyline(polylinePoints).addTo(map);
    // map.setView([37.780012, -122.404827], 18);

    // console.log(line);
});
