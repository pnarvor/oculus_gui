# oculus_gui

Web-based GUI interface for the Blueprint Subsea Oculus front scan sonar.

## Installation

Install the python dependencies:

```
python3 -m pip install --user Django channels["daphne"]
```

Install the web dependencies:

```
./install_webdeps.sh
```

## Usage

This is a web-based interface. A server muistbe started on a machine which can
connect to am Oculus sonar. Then the display can be found using a web brower.

Launch the server:
```
python3 manage.py runserver
```

Then go at this address on a wed browser at this address :

[http://127.0.0.1:8000/oculus_gui/sonar_monitor](http://127.0.0.1:8000/oculus_gui/sonar_monitor).

## Notes

This is a work in progress. A lot of bugs are still in there.




