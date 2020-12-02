

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
}

//function onmessage(...names) {
//    console.log("WebSocket onmessage");
//    inspect_arguments(names);
//}
function onmessage(msg) {
    console.log("WebSocket onmessage");
    console.log(JSON.parse(msg.data))
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
    websocket = new WebSocket('ws://' + window.location.host
                              + '/ws/reconfigure_client/oculus_sonar/');
    websocket.onopen    = onopen;
    websocket.onmessage = onmessage;
    websocket.onerror   = onerror;
    websocket.onclose   = onclose;

    //console.log(websocket);
});
