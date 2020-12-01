

var websocket;

function inspect_arguments(args) {
    for(let i = 0; i < args.length; i++) {
        console.log("Parameter");
        console.log(args[i]);
    }
}

function onopen(...names) {
    console.log("WebSocket onopen");
    inspect_arguments(names);
    
    websocket.messageCount = 0;
    websocket.send("I am opened");
}

function onmessage(...names) {
    console.log("WebSocket onmessage");
    inspect_arguments(names);
    websocket.messageCount++;
    console.log("message count : ", websocket.messageCount);
    
    if(websocket.messageCount < 30)
        websocket.send("I am opened");
    else
        websocket.close(3257)
}

function onerror(...names) {
    console.log("WebSocket onerror");
    inspect_arguments(names);
}

function onclose(...names) {
    console.log("WebSocket onclose");
    inspect_arguments(names);
}

$(document).ready(function() {
    websocket = new WebSocket('ws://' + window.location.host + '/ws/websocket_test/');
    websocket.onopen    = onopen;
    websocket.onmessage = onmessage;
    websocket.onerror   = onerror;
    websocket.onclose   = onclose;

    //console.log(websocket);
});
