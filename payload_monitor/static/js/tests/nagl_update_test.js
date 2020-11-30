

class DataListener
{
    constructor(divContainer)
    {
        this.container     = divContainer;

        this.websocket = new WebSocket(
            'ws://' + window.location.host + '/ws/subscribe/');
        this.websocket.target = this;
        this.websocket.onmessage = function(e) {
            this.target.update(e);
        };

        this.updateCount = 0;

        console.log('Built DataListener');
    }

    async update(e)
    {
        let content = JSON.parse(e.data);
        let metadata = undefined
        if(content.metadata != 'None') {
            metadata = JSON.parse(content.metadata);
        }
        if(content.type == 'cached_data') {
            let request = new XMLHttpRequest();
            request.open("GET", "/payload_monitor/get_cached_data/" + content.data_uuid, true);
            request.responseType = "arraybuffer";
            request.onload = function(e) {
                let data = new Uint8Array(request.response);
                console.log("Got " + data.length.toString() + " bytes of data.");
            };
            request.send(null);
        }
    }

    callback(metadata, binary_data) {
        console.log("Here");
    }
};

$(document).ready(function() {
    $('#status_div')[0].listener = new DataListener($('#status_div')[0]);
});
