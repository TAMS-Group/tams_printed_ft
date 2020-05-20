#!/bin/bash
if [ -f "ft-pushing-box.scad" ];
then 
  echo "Exporting updated parts of the force-sensing pushing box, please wait..."
else
  echo "File 'ft-pushing-box.scad' not found!"
  echo "Please run this script from the openscad directory."
  exit 1 
fi


# viewport tran = 8.07 13.21 -42.06 rot = 262.20 0.0 139.70 dist = 393.66
# --camera=translatex,y,z,rotx,y,z,dist | \ [ --autocenter ] \ [ --viewall ] \
#             [ --imgsize=width,height ] [ --projection=(o)rtho|(p)ersp] \
#             [ --render | --preview[=throwntogether] ] \
#echo "... preview image..."
#openscad -o ft-pushing-box.png --preview --imgsize=800,600 --camera=29.1,90.5,107.5,35.0,0.0,26.5,463 --projection=p \
#             -Dmake_test_box=0 -Dmake_bottom_plate=0 -Dmake_sensor_carrier=0 \
#             -Dmake_coins_holder=0 -Dmake_spring_block=0 -Dmake_fin_carrier=0 -Dshow_sensor_module=1 \
#             -Dmake_asssembled_pushing_box=1 -Dfn=100 ft-pushing-box.scad
#
#echo "... bottom plate ..."
#openscad -o ft-pushing-box-bottom-plate.stl \
#             -Dmake_test_box=0 -Dmake_assembled_pushing_box=0 \
#             -Dmake_bottom_plate=1 -Dmake_sensor_carrier=0 \
#             -Dmake_coins_holder=0 -Dmake_spring_block=0 -Dmake_fin_carrier=0 -Dshow_sensor_module=0 \
#             -Douter_wall_height=15 \
#             -Dfn=100 ft-pushing-box.scad
#
#echo "... sensor carrier..."
#openscad -o ft-pushing-box-sensor-carrier.stl \
#             -Dmake_test_box=0 -Dmake_assembled_pushing_box=0 \
#             -Dmake_bottom_plate=0 -Dmake_sensor_carrier=1 \
#             -Dmake_coins_holder=0 -Dmake_spring_block=0 -Dmake_fin_carrier=0 -Dshow_sensor_module=0 \
#             -Dfn=100 ft-pushing-box.scad
#
#echo "...coins holder..."
#openscad -o ft-pushing-box-coins-holder.stl \
#             -Dmake_test_box=0 -Dmake_assembled_pushing_box=0 \
#             -Dmake_bottom_plate=0 -Dmake_sensor_carrier=0 \
#             -Dmake_coins_holder=1 -Dmake_spring_block=0 -Dmake_fin_carrier=0 -Dshow_sensor_module=0 \
#             -Dfn=100 ft-pushing-box.scad
#
#echo "...fin carrier..."
#openscad -o ft-pushing-box-fin-carrier.stl \
#             -Dmake_test_box=0 -Dmake_asembled_pushing_box=0 \
#             -Dmake_bottom_plate=0 -Dmake_sensor_carrier=0 \
#             -Dmake_coins_holder=0 -Dmake_spring_block=0 -Dmake_fin_carrier=1 -Dshow_sensor_module=0 \
#             -Dfn=100 ft-pushing-box.scad

echo "...spring block..."
openscad -o ft-pushing-box-spring-block-v3.stl \
             -Dmake_test_box=0 -Dmake_assembled_pushing_box=0 \
             -Dmake_bottom_plate=0 -Dmake_sensor_carrier=0 \
             -Dmake_coins_holder=0 -Dmake_spring_block=1 -Dmake_fin_carrier=0 -Dshow_sensor_module=0 \
             -Dfn=100 ft-pushing-box.scad

echo "...spring block..."

echo "OpenSCAD export ok."
