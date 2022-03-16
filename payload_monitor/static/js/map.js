class GpsTrace
{
    constructor(map, maxBufferLength = 10)
    {
        this.map = map;
        this.maxBufferLength = maxBufferLength;

        this.buffer = []
        this.trace  = undefined;
        this.marker = undefined;
    }

    update(lat, lon)
    {
        // updating marker position
        if(typeof this.marker === 'undefined') {
            this.marker = L.marker([lat, lon]).addTo(this.map);
        }
        
        // undating buffer
        this.buffer.push([lat, lon]);

        if(this.buffer.length < this.maxBufferLength) {
            return;
        }

        // buffer is full. Updating displayed trace.
        if(typeof this.trace === 'undefined') {
            this.trace = L.polyline(this.buffer).addTo(this.map);
        }
        else {
            for(let point of this.buffer) {
                this.trace.addLatLng(point);
            }
        }
        this.marker.setLatLng(this.buffer[this.buffer.length - 1]);
        
        this.buffer = [];
    }
};

class GpsDisplay
{
    constructor(mapDiv, topicNames)
    {
        this.mapDiv = mapDiv;
        this.mapDiv.gpsDisplay = this;

        this.build_map();
        this.receivedOne = false;

        this.traces = {};
        
        this.subscribers = []
        //this.nmeaParsers = []
        for(let topicName of topicNames) {
            let subscriber = new RosTopicListener(topicName, 'hemisphere_v500/StampedString');
            let nmeaParser = new GPS;
            subscriber.callbacks.push(this.gps_callback.bind(this, nmeaParser));
            this.subscribers.push(subscriber);
        }
    }

    async gps_callback(parser, content)
    {
        let data = JSON.parse(content.content.scalars);
        parser.update(data.data);

        if(data.data.includes("GGA")) {
            let topicName = content.content.topic;

            let lat = parser.state.lat;
            let lon = parser.state.lon;
            
            if((topicName in this.traces) === false) {
                this.traces[topicName] = new GpsTrace(this.map);
            }
            this.traces[topicName].update(lat, lon);

            if(!this.receivedOne) {
                this.map.setView({lat : lat, lon : lon}, 18);
                this.receivedOne = true;
            }
        }
    }

    build_map(firstViewTarget = {lon: 0, lat: 0},
              firstZoom = 2)
    {
        this.map = L.map('map').setView(firstViewTarget, firstZoom);
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
