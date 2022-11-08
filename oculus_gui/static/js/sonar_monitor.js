
async function fetch_data_test(dataInfo) {
    let metadata = dataInfo.content.scalars;
    let data = new Uint8Array(await dataInfo.fetch_cached_data('data'));
    console.log(data);
}



$(document).ready(function() {
    let displayDiv = $(".sonar-gui")[0];
    console.log(displayDiv);
    
    let dataFetcher = new DataListener('/ws/sonar_data/');
    displayDiv.dataFetcher = dataFetcher;

    dataFetcher.add_callback(fetch_data_test)
});
