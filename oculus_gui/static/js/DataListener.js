class DataInfo
{
    constructor(content) {
        this.content = content;
    }

    async fetch_cached_data(name)
    {
        // Two requests are sent to the server, the data is transfered twice.
        // (investigate)
        let content = this.content.vectors[name];
        return new Promise(function(resolve, reject) {
            let request = new XMLHttpRequest();
            request.open("GET", content.cache_request_uri + content.data_uuid, true);
            request.responseType = "arraybuffer";
            request.onload = function(e) {
                if(request.status != 200)
                    reject("Cached data fetch error");
                resolve(request.response);
            };
            request.send(null);
        });
    }
};

class DataListener
{
    constructor(subscriptionUrl = '/ws/subscribe/')
    {
        this.callbacks = []
        this.websocket = new WebSocket(
            'ws://' + window.location.host + subscriptionUrl);
        this.websocket.target = this;
        this.websocket.onmessage = function(e) {
            this.target.update(e);
        };
    }

    async update(e)
    {
        let data = new DataInfo(JSON.parse(e.data));
        for(const callback of this.callbacks) {
            callback(data);
        }
    }

    add_callback(callback) {
        this.callbacks.push(callback)
    }
};

