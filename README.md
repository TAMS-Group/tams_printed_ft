# tams_printed_ft

This repository provides OpenSCAD 3D models together with the corresponding software
to build and assemble a family of low-cost 3D printed force and force/torque sensors.
While multi-axis force/torque sensors are readily available commercially from several vendors,
these a low-volume high-precision devices, and the corresponding costs and complex integration
have so far limited their wide deployment.

Our design approach combines 3D printable plastic structures with commodity optical sensors
and standard Arduino-style microcontrollers. The general sensor configuration, including
type and number of sensors, sensor layout, and structure of the elastic element can be chosen
freely according to the task. We provide a couple of example designs that have been optimized
for 3D printability using common FFF/FDM (fused filament fabrication / fused deposition modeling)
3D machines. The electronics can use custom circuit boards, but many of the sensors can also
be hand-wired without problem, and a working 6-axis force/torque sensor can be built for less
than 20 US dolloars, plus a few hours of 3D printing.

!["3D printed f/t sensors"](/doc/images/teaer.jpg)

## Note: stub while paper under review

This repository is currently a stub, as the journal paper describing the general approach
and several key algorithms is still under review. If you are a reviewer, please don't hesitate
to contact the authors by email for a preview of the final repository contents.

## Getting started

The following instructions assume that you want to build and use a force/torque sensor 
together with ROS. If so, please proceed as follows:

0. read this README and check the overview paper

1. clone this repository into your catkin workspace, then build your workspace:

'''
cd ~/catkin_ws/src
git clone github.com/TAMS-Group/tams_printed_ft
cd ~/catkin_ws
catkin_make
'''

2. check the OpenSCAD and/or meshes subdirectories and select the sensor that you need.
Run OpenSCAD and edit the existing parameters or completely modify the sensor geometry.
Export the individual parts to STL meshes.

3. Use your favorite slicing software to convert all needed mechanical parts into G-Code
for your 3D-printer, then start printing the parts.

4. Check that you have the required electronics components (optical sensors, microcontroller,
pullup-resistors, capacitors, ...) and the mechanical components (screws and nuts).

5. Assemble the mechanical parts, insert the optical sensors or ready-made circuit boards,
and wire the optical sensors to the microcontroller/amplifier boards.

6. Connect the microcontroller to the host computer and upload/flash the corresponding
firmware. Modify the firmware according to your needs. Once powered up and flashed,
the microcontroller should start sending out raw sensor data, 
using the selected communication protocol (e.g. I2C/TWI, SPI, UART, USB),
baudrate (e.g. 115200 baud), etc.

7. Edit and update the launch files to match your setup (e.g. name of the root tf frame,
name of your robots and F/T sensor frames, sample rates, comunication device, etc.).

8. Run the ROS driver for the selected sensor, and check that the raw data looks ok:
```
roscore
roslaunch tams_printed_ft bottle_ft.launch  [device:=/dev/ttyUSB2 ...]
rosrun plotjuggler PlotJuggler -> start streaming -> /printed_ft/rawdata -> plot
```

9. Record calibration data
```
roslaunch tams_printed_ft bottle_ft_calibration_helper.launch
run experiment ... press control-c
python3 scripts/linear_bottle.py 
check calibration quality, select best algorithm, ...
cp /tmp/bottle_ft_huber_1e-4.yaml config/bottle_ft_calibration.yaml
```

10. Restart the sensor to use the recently created calibration file

## Directory Structure

What is in here? The basic structure of this repository is a common ROS catkin package, with a few extra directories for the OpenSCAD models and Arduino firmware:


* `config`: ROS configuration files, including example sensor calibration YAML files and rviz / plotjuggler configurations;
* `doc`: Misc documentation, including the overview paper, datasheets for the optical sensors, Arduino pinouts, and
  some images/screenshots;
* `firmware`: Arduino/Teensy source code for the sensor firmware; copy (or symlink) into your `~/Arduino/` folder to use.
* `launch`: ROS launch files for the sensor drivers and utilities;
* `meshes`: this subdirectory holds STL meshes ready for 3D printing for some of the sensors, as well as Collada .dae files needed for the ROS URDF models;
* `openscad`: Configurable OpenSCAD source code for some of our sensors. Most of the files will render an assembled model of the complete sensor, but individual parts can be enabled or disabled using the provided boolean variables. This directory also provides automated export scripts to generate 3D STL meshes for the individual 3D printable files;
* `nodes`: ROS Python driver nodes for the different sensors, as well as calibration scripts and plotting utilities;
* `scripts`: misc shell scripts
* `src`: ROS C++ source files, currently the calibration-helper tool and the radial-pattern generator;
* `urdf`: ROS URDF models for the sensors;
* `package.xml` and `CMakeLists.txt`: the common ROS catkin package description and build files.

Note that there are no header files (yet).

## Prototype sensors

* !["Robocup soccer foot sensor"](/doc/images/foot.jpg)
  A foot force sensor for a Robocup soccer humanoid;
* !["2-DOF sensor module"](/doc/images/2DOF-sensor-module.jpg)
  A convenient sensor module built from two fork-type photointerrupters, used as a basic components
  in several of our designs;
* !["spacemouse six-axis F/T sensor"](/doc/images/spacemouse.png) 
  A small six-sensor six-axis force/torque sensor with very light force limit (about ~4N);
* !["adjustable six-axis F/T sensor"](/doc/images/adjustable-base-ring.jpg)
   A large six-axis force/torque sensor, using four 2-DOF sensor modules and grub-screws for initial zero-ing;
* !["bottle-ft ring-type six-axis F/T sensor"](/doc/images/bottle-ft-pouring.jpg)
   A ring-type six-axis force/torque sensor, using four 2-DOF sensor modules.


## OpenSCAD designs and 3D printing

The sensors presented in the paper were all designed using the OpenSCAD software.
We recommend to use OpenSCAD to edit, update, and configure the basic sensor designs to your needs,
but we also provide a set of ready-made STL meshes ready for slicing and printing on your 3D printer.
Most of the meshes are optimized for 3D printability without support structures, 
just make sure to select the appropriate mesh orientation for printing (which should be obvious).
A few STL models contain overhanging parts and bridges; please check for the best mesh orientation
and experiment with different printer settings (including support structures) to find a suitable setup.

## Arduino firmware

While our example designs intentionally use different microcontrollers, 
the core program logic is always the same.
The basic hardware setup consists of a stabilized power supply for the infrared LEDs,
while the phototransistors are connected to pullup resistors and also directly to 
the analog input pins of the selected microcontrollers.

The program first configures the microcontroller output and input pins,
sets up and initializes the communication with the host computer 
and creates the data structures for raw-data filtering.
Communication is typically serial over USB, but also native USB when supported by the controller,
or UDP/WIFI for the wireless boards.

The main loop of the program then repeatedly reads the active analog input pins,
typically staring with one cold read to ensure that the analog voltage has stabilized,
followed by a few AD conversions to digitize the current illumination level.
Optionally, oversampling and filtering is applied.
Where applicable, LEDs might also be pulsed to reduce average power consumption and
to support dark current detection and measurement.

Raw output values are then sent to the host PC, together with sequence number and
health/error indication (overload, ambient light detection, breakage, etc.).
Due ensure high sample rates and low latencies, conversion to floating point 
and calibration is NOT performed on the microcontrollers.

## ROS drivers



## URDF models

We also provide ROS URDF models for some of the sensors.
The modeling approach combines the actual (high-res) 3D meshes also used for 3D printing
for the visual model, while a simplified geometric model (cylinder, box) is used for
the sensor collision model.

For the small sensors, the tilt under load cannot be ignored, and is modeled 
using a generic a six-DOF joint between the base (robot side) and swiveling (tool side)
parts of the sensor. Corresponding yaml configuration files are provided for Gazebo.

## Sensor calibration

As described in the paper, all standard approaches can be used to calibrate the sensor(s),
and Python scripts are provided to calculate calibration matrices for a tested sensor.
Calibration against a second reference sensor is of course easiest, and typically only
requires to edit and configure the geometric layout between the 3D printed and the existing
reference sensor in the sensor launch file(s).



