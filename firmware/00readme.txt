This directory collects microcontroller firmware for our prototype 
sensors. The software was developed using the Arduino IDE and follows
the Arduino file system convention, with one subdirectory for each
sensor prototype, and the corresponding C++ sources (*.ino *.c)
and headers (*.h) inside and below that subdirectory.


1)
The most recent version of the Arduino IDE can be downloaded from
the Arduino website; third-party microcontrollers can sometimes be
integrated vai the Arduino board-manager, but others (e.g. Teensy)
need external software:

Arduino software homepage:
https://https://www.arduino.cc/en/Main/Software

PJRC "Teensyduino" plugin/programmer download:
https://www.pjrc.com/teensy/td_download.html


2)
After installing the Arduino IDE, you may want to copy (or symlink) 
the contents of this directory to your $HOME/Arduino folder, 
To make the files available to your Arduino IDE installation. 
Of course, you don't need to copy files for sensors that you don't build:

# deep copy of all directories:
#
roscd tams_printed_ft 
cp -r firmware/* ~/Arduino

# or symlink one directory:
#
roscd
ln -s `pwd`/firmware/bottle_ft ~/Arduino/


3)
The basic algorithm used here is always the same: we first configure
the microcontroller, and then enter and endless loop that reads all
analog phototransistor inputs and communicates with the host.

Setup includes:
- basic configuration of the microcontroller used
- setup of the analog input channels (e.g. ADC resolution, pullups, ...)
- setup of the digital I/Os, e.g. outputs for multiplexing the infrared LEDs
- setup of external sensors, e.g. IMU
- setup of host communiciton, typically USB or RS232, but also Wifi/UDP etc.

Loop includes:
- reading analog channels (phototransistor currents)
- consistency checks on the values, including overload detection
- reading external sensors (e.g. IMU)
- sending the sensor values to the host
- optional sleep() to reach the configured sample rate


4)
The default ADC settings encoded in the Arduino IDE are quite conservative,
so we sometimes modify the ADC settings (ADC clock, filtering) to reach 
higher sample rates of better filtering.
- 
