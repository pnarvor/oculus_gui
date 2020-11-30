
callback_test = async function(data)
{
    let cachedData = await data.fetch_cached_data();
    console.log((new Uint8Array(cachedData)).length);
}

$(document).ready(function() {
    let listener = new DataListener();
    listener.callbacks.push(callback_test);

    $('#status_div')[0].listener = listener;
});
