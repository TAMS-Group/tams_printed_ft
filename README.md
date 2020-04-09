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

## Note: stub while paper under review

This repository is currently a stub, as the journal paper describing the general approach
and several key algorithms is still under review. If you are a reviewer, please don't hesitate
to contact the authors by email for a preview of the final repository contents.

## Getting started

## Prototype sensors




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



