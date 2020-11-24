

class PingDisplay
{
    constructor(divContainer)
    {
        this.container = divContainer;

        this.websocket = new WebSocket(
            'ws://' + window.location.host + '/ws/ping_display/');
        this.websocket.target = this;
        this.websocket.onmessage = function(e) {
            this.target.update(e);
        };

        console.log('Build PingDisplay');
    }

    update(e)
    {
        //console.log("Got data");
        console.log(e.data);
    }
};

$(document).ready(function() {
    $('#status_div')[0].pingDisplay = new PingDisplay($('#status_div')[0]);
});
