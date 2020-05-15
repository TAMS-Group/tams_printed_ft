OpenSCAD 3D models and STL files for the printed_ft prototype sensors.

This directory holds the actual 3D CAD files and the external
component libraries ("vitamins") used to design and print our
prototype sensors. 

* '00readme.txt' - this file.

Visit https://www.openscad.org for the OpenSCAD homepage, 
documentation and software download links.

Vitamins / libraries are:

* `optokoppler.scad' - has 3D models of the optical sensors
  used in our prototypes; some of the modules provide options
  to enable/disable wires or convex-hull variants of the device,
  please check the source code for the available parameters.
  For example, rendering a fork-type sensor using its convex hull
  might help to create a matching cut-out for the device.

* 'electronics.scad' - a few rough 3D models of Arduino and
  Teensy microcontroller boards liked by us.

* 'breadboard.scad' - parameteric model of typical soldering
  breadboards; this can help for quick prototyping when you
  don't want to design and route a custom circuit board.

About the sensor 3D models:

A single self-contained OpenSCAD file is provided for each of
the prototype sensors. For most of the sensors, the complete 
3D model is split into individual parts that are optimized
for 3D printability and easy assembly of the components.

By default, the OpenSCAD files are configured to render the
complete (aka "assembled") sensor for a quick overview,
plus a set of enable variables to render and export the 
individual pieces. This can be done interactively from the
OpenSCAD editor, or automatically using the provided export scripts.
These scripts define the corresponding variables and then call
OpenSCAD repeatedly to render the individual parts and to export
STL models ready for 3D printing.

Almost all parts are designed for printing without support
structures on common FDM/FFF printers, typically with the
z-axis pointing up, but a few parts need re-orientation on
the printbed for best results.

Part dimensions are using millimeter units, suitable for most
3D slicing and printing software.  If necessary, a simple call of 
the 'scale()' operator can help to resize the parts as needed.
Note that ROS in particular expects all dimensions in meters,
so scale your meshes by 0.001 before exporting for use in ROS
URDF/xacro models. Also note that ROS expects binary-format STLs,
while OpenSCAD only exports ASCII-format STLs; use an external
tool (e.g. Meshlab or blender) to convert from ASCII-format STL
to binary format or Collada.

  
  
 




