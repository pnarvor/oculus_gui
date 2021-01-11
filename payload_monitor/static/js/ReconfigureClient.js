

class ReconfigureClient
{
    constructor(target)
    {
        this.target    = target;
        this.targetUrl = 'ws://' + window.location.host
                       + '/ws/reconfigure_client/' + target + '/';
        this.websocket = new WebSocket(this.targetUrl);
        this.websocket.onmessage = this.on_message_base.bind(this);
    }

    on_message_base(msg) {
        // here "this" refers to the websocket
        //console.log(msg)
        let data = JSON.parse(msg.data);
        if(data.type === "config") {
            this.on_config(data.payload);
        }
        else if(data.type === "description") {
            this.configDescription = data.payload;
            this.on_description(this.configDescription);
        }
        else {
            console.log("Unknown message type");
            console.log(data);
        }
    }

    on_config(msg) {
        console.log("Got config");
        console.log(msg);
    }

    on_description(msg) {
        console.log("Got description");
        console.log(msg);
    }
};
