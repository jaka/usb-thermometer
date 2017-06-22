# usb-thermometer

Read temperature from USB thermometer (with DS18B20 1–wire probe).

Supported devices:
* [KEL USB thermometer](http://kel.si/)

### How to run it

1. Clone (or download) this repository (and extract files)
2. Run `$ make` to compile the binary
3. Run the binary: `$ ./usbtemp`

The output is in degrees Celsius and looks like:
```
May 13 17:05:02 Sensor C: 22.62
```

Date/time formatting is `%b %d %H:%M:%S`.

### Licence

Copyright 2017, jaka

Software is released under the 2-Clause BSD License.
