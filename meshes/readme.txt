Collada and STL format 3D meshes for the ROS URDF models.

Do not use these meshes for 3D printing, unless you know
what you are doing. Instead, please check and use the 
individual parts meshes in the "openscad/stl" folder or
use OpenSCAD to configure and modify the prototype templates
and export your own STL files.

Following the usual ROS package structure, this directory
contains the Collada and STL format 3D meshes of the URDF
sensor models defined in the "urdf" subdirectory. Most of
these meshes are in fact combinations of several parts,
partitioned for efficient simulation, but not partitioned
for 3D-printability.
