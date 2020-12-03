#! /bin/bash

# Will download required resources.

set -e

# This will remove all files download in this script
if [ "$1" = "uninstall" ] || [ "$1" = "-u" ]; then
    rm -rv payload_monitor/static/js/libs
    rm -rv payload_monitor/static/css/libs
    exit 1
fi


# Creating resources folder if they do not exist
if [ ! -d "payload_monitor/static/js/libs" ]; then
    mkdir -p payload_monitor/static/js/libs
fi
if [ ! -d "payload_monitor/static/css/libs" ]; then
    mkdir -p payload_monitor/static/css/libs
fi

# jquery
wget -O payload_monitor/static/js/libs/jquery.js https://cdnjs.cloudflare.com/ajax/libs/jquery/3.5.0/jquery.js

# materialize
wget -O payload_monitor/static/css/libs/materialize.css https://cdnjs.cloudflare.com/ajax/libs/materialize/1.0.0/css/materialize.css
wget -O payload_monitor/static/js/libs/materialize.js   https://cdnjs.cloudflare.com/ajax/libs/materialize/1.0.0/js/materialize.js

# material-icons
if [ ! -d "payload_monitor/static/css/libs/icons" ]; then
    mkdir -p payload_monitor/static/css/libs/icons
fi
wget -O payload_monitor/static/css/libs/material-icons.css https://fonts.googleapis.com/icon?family=Material+Icons
wget -O payload_monitor/static/css/libs/icons/MaterialIcons-Regular.ttf 'https://github.com/google/material-design-icons/blob/master/font/MaterialIcons-Regular.ttf?raw=true'
sed  -i 's/src.*/src:url("icons\/MaterialIcons-Regular.ttf") format("truetype");/g' payload_monitor/static/css/libs/material-icons.css

# Roboto font
if [ ! -d "payload_monitor/static/css/libs/fonts" ]; then
    mkdir -p payload_monitor/static/css/libs/fonts
fi
wget https://www.fontsquirrel.com/fonts/download/roboto
unzip roboto -d payload_monitor/static/css/libs/fonts
rm roboto

# # plotly
# wget -O payload_monitor/static/js/libs/plotly.js https://cdn.plot.ly/plotly-latest.js

# three
wget -O payload_monitor/static/js/libs/three.js https://raw.githubusercontent.com/mrdoob/three.js/dev/build/three.js
wget -O payload_monitor/static/js/libs/OrbitControl.js https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/js/controls/OrbitControls.js

# wget -O payload_monitor/static/js/libs/pixi.js https://pixijs.download/release/pixi.js
