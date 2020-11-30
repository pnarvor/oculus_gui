

class PingListener
{
    constructor(divContainer)
    {
        this.container     = divContainer;

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
    }
};

$(document).ready(function() {
    $('#status_div')[0].listener = new PingListener($('#status_div')[0]);
});
