#! /bin/bash

# Will download required resources.

set -e

target=oculus_gui

# This will remove all files download in this script
if [ "$1" = "uninstall" ] || [ "$1" = "-u" ]; then
    rm -rv $target/static/js/libs
    rm -rv $target/static/css/libs
    exit 1
fi


# Creating resources folder if they do not exist
if [ ! -d "$target/static/js/libs" ]; then
    mkdir -p $target/static/js/libs
fi
if [ ! -d "$target/static/css/libs" ]; then
    mkdir -p $target/static/css/libs
fi

# jquery
wget -O $target/static/js/libs/jquery.js https://cdnjs.cloudflare.com/ajax/libs/jquery/3.5.0/jquery.js

# materialize
wget -O $target/static/css/libs/materialize.css https://cdnjs.cloudflare.com/ajax/libs/materialize/1.0.0/css/materialize.css
wget -O $target/static/js/libs/materialize.js   https://cdnjs.cloudflare.com/ajax/libs/materialize/1.0.0/js/materialize.js

# material-icons
if [ ! -d "$target/static/css/libs/icons" ]; then
    mkdir -p $target/static/css/libs/icons
fi
wget -O $target/static/css/libs/material-icons.css https://fonts.googleapis.com/icon?family=Material+Icons
wget -O $target/static/css/libs/icons/MaterialIcons-Regular.ttf 'https://github.com/google/material-design-icons/blob/master/font/MaterialIcons-Regular.ttf?raw=true'
sed  -i 's/src.*/src:url("icons\/MaterialIcons-Regular.ttf") format("truetype");/g' $target/static/css/libs/material-icons.css

# Roboto font
if [ ! -d "$target/static/css/libs/fonts" ]; then
    mkdir -p $target/static/css/libs/fonts
fi
wget https://www.fontsquirrel.com/fonts/download/roboto
unzip roboto -d $target/static/css/libs/fonts
rm roboto

# # plotly
# wget -O $target/static/js/libs/plotly.js https://cdn.plot.ly/plotly-latest.js

# three
wget -O $target/static/js/libs/three.js https://raw.githubusercontent.com/mrdoob/three.js/dev/build/three.js
wget -O $target/static/js/libs/OrbitControl.js https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/js/controls/OrbitControls.js

# wget -O $target/static/js/libs/pixi.js https://pixijs.download/release/pixi.js

# leaflet
wget -O $target/static/js/libs/leaflet.js https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/leaflet.js
wget -O $target/static/css/libs/leaflet.css https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/leaflet.css


# GPS.js (for parsing NMEA strings)
wget -O $target/static/js/libs/gps.js https://raw.githubusercontent.com/infusion/GPS.js/master/gps.js
