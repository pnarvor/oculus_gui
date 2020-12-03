class DataInfo
{
    constructor(content) {
        this.content = content;
    }

    async fetch_cached_data()
    {
        // Two requests are sent to the server, the data is transfered twice.
        // (investigate)
        const content = this.content
        return new Promise(function(resolve, reject) {
            if(content.type != 'cached_data') {
                reject("no cached data to fetch");
            }

            let request = new XMLHttpRequest();
            request.open("GET", content.cache_request_url + content.data_uuid, true);
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
        //let content = JSON.parse(e.data);
        ////let metadata = undefined
        ////if(content.metadata != 'None') {
        ////    metadata = JSON.parse(content.metadata);
        ////}
        //let callbacks = this.callbacks;
        //if(content.type == 'cached_data') {
        //    let request = new XMLHttpRequest();
        //    request.open("GET", content.cache_request_url + content.data_uuid, true);
        //    request.responseType = "arraybuffer";
        //    request.onload = function(e) {
        //        for(const callback of callbacks) {
        //            callback(content, request.response);
        //        }
        //    };
        //    request.send(null);
        //}
        //else {
        //    for(const callback of callbacks) {
        //        callback(content, undefined);
        //    }
        //}
    }
};

