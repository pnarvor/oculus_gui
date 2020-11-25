

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

        this.updateCount = 0;

        console.log('Built PingDisplay');
    }

    async update(e)
    {
        //console.log(e.data);

        //console.log("Got data", this.updateCount);
        this.updateCount++;

        
        // this creates an untyped ArrayBuffer (hopefully view).
        // await Reponse needed because ?
        let buffer = await new Response(e.data).arrayBuffer();
        let data   = new Uint8Array(buffer);

        let metadataSize = new DataView(buffer).getUint32(0, true);
        let metadata = JSON.parse(new TextDecoder("utf-8").decode(
            data.slice(4, 4 + metadataSize)));
        let raw_data = data.slice(4 + metadataSize); // until the end
        
        this.update_display(metadata, raw_data);
    }
    
    update_display(metadata, raw_data)
    {
        console.log(metadata);
        console.log(raw_data);
    }
};

$(document).ready(function() {
    $('#status_div')[0].pingDisplay = new PingDisplay($('#status_div')[0]);
});
