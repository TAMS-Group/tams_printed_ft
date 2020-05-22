#!/bin/bash
if [ -f "adjustable-ft-v1.scad" ];
then 
  echo "Exporting individual parts of the force-sensor, please wait..."
else
  echo "File 'adjustable-ft-v1.scad' not found!"
  echo "Please run this script from the openscad directory."
  exit 1 
fi


# viewport tran = 8.07 13.21 -42.06 rot = 262.20 0.0 139.70 dist = 393.66
# --camera=translatex,y,z,rotx,y,z,dist | \ [ --autocenter ] \ [ --viewall ] \
#             [ --imgsize=width,height ] [ --projection=(o)rtho|(p)ersp] \
#             [ --render | --preview[=throwntogether] ] \
echo "... preview image..."
openscad -o adjustable-ft.png --preview --imgsize=1000,800 --camera=26.25,24.15,18.32,71.8,0.00,35.0,401.52 --projection=p \
             -Dfn=100 adjustable-ft-v1.scad

echo "... base ring..." 
openscad -o adjustable-ft-v1-base-ring.stl \
             -Dmake_base_ring=1 \
             -Dmake_upper_ring=0 \
             -Dmake_fin_ring=0 \
             -Dmake_distance_ring=0 \
             -Dshow_sensors=0 \
             -Dshow_arduino_nano=0 \
             -Dfn=100 adjustable-ft-v1.scad

echo "... upper ring..." 
openscad -o adjustable-ft-v1-upper-ring.stl \
             -Dmake_base_ring=0 \
             -Dmake_upper_ring=1 \
             -Dmake_fin_ring=0 \
             -Dmake_distance_ring=0 \
             -Dshow_sensors=0 \
             -Dshow_arduino_nano=0 \
             -Dfn=100 adjustable-ft-v1.scad

echo "... fin ring..." 
openscad -o adjustable-ft-v1-fin-ring.stl \
             -Dmake_base_ring=0 \
             -Dmake_upper_ring=0 \
             -Dmake_fin_ring=1 \
             -Dmake_distance_ring=0 \
             -Dshow_sensors=0 \
             -Dshow_arduino_nano=0 \
             -Dfn=100 adjustable-ft-v1.scad

echo "... distance ring..." 
openscad -o adjustable-ft-v1-distance-ring.stl \
             -Dmake_base_ring=0 \
             -Dmake_upper_ring=0 \
             -Dmake_fin_ring=0 \
             -Dmake_distance_ring=1 \
             -Dshow_sensors=0 \
             -Dshow_arduino_nano=0 \
             -Dfn=100 adjustable-ft-v1.scad

echo "OpenSCAD export ok."
