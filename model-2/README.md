
# Description

This sensor was done in 2015, with few evolution from model 1.

The sensors are RFM12B 868Mhz radio transceivers.

One central transceiver receives all measures from all remote sensors.  

# Hardware

Receiver listens on USB tty port:

![JeeLink](/res/jeelink.jpg "JeeLink")

Plenty emmiter send data using below as base board.

![Arduino pro mini](/res/arduino-pro-mini.jpg "Arduino Pro Mini 3.3v")

Plenty sensors as seen on main page.

Wiring sensors is described at [arduino-res](https://github.com/kalemena/arduino-res)


## Breadboard design

Basically initial boards are done like below:

![Wiring Breadboard 3xAAA](/res/20150104_211916-notes.JPG "Wiring Breadboard 3xAAA")

![Wiring Breadboard 1xAAA](/res/20150104_211827-notes.JPG "Wiring Breadboard 1xAAA")

## PCB design

Going from Breadboard to PCB using simple double side PCB:

![Breadboard](/res/avant.png "Breadboard")

PCB sample look like below:

![PCB](/res/apres.jpg "PCB")

# Implementation

## How-To - central receiver node

* Open sketch from folder 'src/sketches/sensor_rx_unit/' into Arduino IDE

* Upload sketch to radio-enabled device

* Connect receiver node to USB and find which usb resource connected

```js
$ lsusb
$ dmesg
```

* Allow user access to resource and set speed

```js
$ sudo stty -F /dev/ttyUSB0 57600
$ sudo chmod 777 /dev/ttyUSB0
$ cat /dev/ttyUSB0
```

* Edit test script to point to USB devices or point Node-Red to correct USB device.


## How-To - remote sensor nodes

* Open sketch from folder 'src/sketches/sensor_trx_unit/' into Arduino IDE

* Modify sketch to match sensor devices plugged onto the arduino (ensure pins are correct)

* Upload sketch to radio-enabled device

* ... let it submit metrics over the air ...

## Tests

A small javascript code is done to grab on tty USB port and submit to mqtt queue.

Go to folder 'src/js/' and run below commands:

```js
$ npm install serialport
$ npm install mqtt
$ node pub-sensors.js
```
