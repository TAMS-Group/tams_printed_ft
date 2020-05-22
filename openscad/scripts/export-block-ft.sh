#!/bin/bash
if [ -f "block-ft.scad" ];
then 
  echo "Exporting individual parts of the force-sensor, please wait..."
else
  echo "File 'block-ft.scad' not found!"
  echo "Please run this script from the openscad directory."
  exit 1 
fi


# viewport tran = 8.07 13.21 -42.06 rot = 262.20 0.0 139.70 dist = 393.66
# --camera=translatex,y,z,rotx,y,z,dist | \ [ --autocenter ] \ [ --viewall ] \
#             [ --imgsize=width,height ] [ --projection=(o)rtho|(p)ersp] \
#             [ --render | --preview[=throwntogether] ] \
echo "... preview image..."
openscad -o block-ft.png --preview --imgsize=1000,800 --camera=26.25,24.15,18.32,71.8,0.00,35.0,401.52 --projection=p \
             -Dfn=100 block-ft.scad

exit

echo "... M3207 adapter (two parts)..."
openscad -o block-ft-m3207-adapter-outer.stl \
             -Dmake_upper_pcb=0 -Dmake_elastic_member=0 -Dmake_base_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dmake_sunrise_m3207_plate=1 \
             -Dcover_ring_thickness=8 -Dcover_ring_inner_diameter=40 \
             -Dpattern_plate_thickness=1.0 \
             -Dfn=100 block-ft.scad

echo "... elastic member (outer arm width 4)..."
openscad -o block-ft-x-07-10-13-y-07-7-z-06-10-4.stl \
             -Dmake_elastic_member=1 \
             -Dmake_sunrise_m3207_plate=0 -Dmake_base_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dinner_arm_thickness=0.7 -Dinner_arm_length=10.0 -Dinner_arm_separation=13 \
             -Dmiddle_arm_thickenss=0.7 -Dmiddle_arm_length=8 -Dmiddle_arm_height=10.0 \
             -Douter_arm_thickness=0.6 -Douter_arm_length=10.0 -Douter_arm_width=4 \
             -Dfn=100 block-ft.scad

echo "... elastic member (outer arm width 6)..."
openscad -o block-ft-x-07-10-11-y-07-9-z-06-10-6.stl \
             -Dmake_elastic_member=1 \
             -Dmake_sunrise_m3207_plate=0 -Dmake_base_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dinner_arm_thickness=0.7 -Dinner_arm_length=10.0 -Dinner_arm_separation=11 \
             -Dmiddle_arm_thickenss=0.7 -Dmiddle_arm_length=9 -Dmiddle_arm_height=10.0 \
             -Douter_arm_thickness=0.6 -Douter_arm_length=10.0 -Douter_arm_width=6 \
             -Dfn=100 block-ft.scad

echo "... elastic memberi (outer arm width 8)..."
openscad -o block-ft-x-07-10-9-y-07-10-z-06-10-8.stl \
             -Dmake_elastic_member=1 \
             -Dmake_sunrise_m3207_plate=0 -Dmake_base_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dinner_arm_thickness=0.7 -Dinner_arm_length=10.0 -Dinner_arm_separation=9 \
             -Dmiddle_arm_thickenss=0.7 -Dmiddle_arm_length=10 -Dmiddle_arm_height=10.0 \
             -Douter_arm_thickness=0.6 -Douter_arm_length=10.0 -Douter_arm_width=8 \
             -Dfn=100 block-ft.scad

echo "... elastic member..."
openscad -o block-ft-x-07-10-15-y-07-7-z-06-10-6.stl \
             -Dmake_elastic_member=1 \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dinner_arm_thickness=0.7 -Dinner_arm_length=10.0 -Dinner_arm_separation=15 \
             -Dmiddle_arm_thickenss=0.7 -Dmiddle_arm_length=7 -Dmiddle_arm_height=10.0 \
             -Douter_arm_thickness=0.6 -Douter_arm_length=10.0 -Douter_arm_width=6 \
             -Dfn=100 block-ft.scad

echo "... base plate..."
openscad -o block-ft-base-plate.stl \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_ring=0 -Dmake_elastic_member=0 -Dmake_base_plate=1 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dbase_thickness=3 -Dbase_support_pillar_thickness=2 \
             -Dfn=100 block-ft.scad


echo "... cover ring (upper  plate)..."
openscad -o block-ft-cover-ring-40-height-8.stl \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_ring=1 -Dmake_elastic_member=0 -Dmake_base_plate=0 \
             -Dmake_cover_plate=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dcover_ring_thickness=8 -Dcover_ring_inner_diameter=40 \
             -Dcover_screw_cavity=4 \
             -Dfn=100 block-ft.scad

openscad -o block-ft-cover-ring-40-height-9.stl \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_ring=1 -Dmake_elastic_member=0 -Dmake_base_plate=0 \
             -Dmake_cover_plate=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dcover_ring_thickness=9 -Dcover_ring_inner_diameter=40 \
             -Dcover_screw_cavity=5 \
             -Dfn=100 block-ft.scad

openscad -o block-ft-cover-ring-40-height-10.stl \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_plate=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dmake_cover_ring=1 -Dmake_elastic_member=0 -Dmake_base_plate=0 \
             -Dcover_ring_thickness=10 -Dcover_ring_inner_diameter=40 \
             -Dcover_screw_cavity=6 \
             -Dfn=100 block-ft.scad


echo "... pattern plate..."
openscad -o block-ft-pattern-plate-40.stl \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dmake_pattern_plate=1 -Dmake_elastic_member=0 -Dmake_base_plate=0 \
             -Dcover_ring_thickness=8 -Dcover_ring_inner_diameter=40 \
             -Dpattern_plate_thickness=1.0 \
             -Dfn=100 block-ft.scad

echo "... fake upper pcb..."
openscad -o block-ft-fake-pcb-40.stl \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=1 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=0 \
             -Dmake_elastic_member=0 -Dmake_base_plate=0 \
             -Dcover_ring_thickness=8 -Dcover_ring_inner_diameter=40 \
             -Dpattern_plate_thickness=1.0 \
             -Dfn=100 block-ft.scad

echo "... base holder..."
openscad -o block-ft-base-holder.stl \
             -Dmake_sunrise_m3207_plate=0 \
             -Dmake_cover_plate=0 -Dmake_cover_ring=0 -Dmake_fake_pcb=0 -Dmake_upper_pcb=0 \
             -Dmake_pattern_plate=0 -Dmake_spacer_plate=0 -Dmake_base_holder=1 \
             -Dmake_elastic_member=0 -Dmake_base_plate=0 \
             -Dcover_ring_thickness=8 -Dcover_ring_inner_diameter=40 \
             -Dpattern_plate_thickness=1.0 \
             -Dfn=100 block-ft.scad








echo "OpenSCAD export ok."
