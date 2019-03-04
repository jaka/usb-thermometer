# usb-thermometer

Read temperature from USB thermometer (with DS18B20 1–wire probe).

Supported devices:
* [KEL USB Thermometer](https://usbtemp.com/)

### How to run it

1. Clone (or download) this repository (and extract files)
2. Run `$ make` to compile the binary
3. Run the binary: `$ ./usbtemp`

The output is in degrees Celsius and looks like:
```
May 13 17:05:02 Sensor C: 22.62
```

Date/time formatting is `%b %d %H:%M:%S`.

### Troubleshooting

User, running binary, must have permissions to write to `/dev/ttyUSB0` or similar character device.
Usually, `adduser` to `dialout` group or `chmod o+rw` the character device helps.

### Licence

Copyright 2017, jaka

Software is released under the 2-Clause BSD License.
