

class ReconfigureClient
{
    constructor(target)
    {
        this.target    = target;
        this.targetUrl = 'ws://' + window.location.host
                       + '/ws/reconfigure_client/' + target + '/';
        this.websocket = new WebSocket(this.targetUrl);
        this.websocket.onmessage = this.on_message_base;
        this.websocket.target    = this;
    }

    on_message_base(msg) {
        // here "this" refers to the websocket
        //console.log(msg)
        let data = JSON.parse(msg.data);
        if(data.type === "config") {
            this.target.on_config(data.payload);
        }
        else if(data.type === "description") {
            this.target.on_description(data.payload);
        }
        else {
            console.log("Unknown message type");
            console.log(data);
        }
    }

    on_config(msg) {
    }

    on_description(msg) {
    }
};
