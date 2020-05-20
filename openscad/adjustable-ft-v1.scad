/** adjustable-ft-v1.scad
 * 
 * 2020.02.06 - inner ring to match Sunrise M3207 sensor
 * 2020.02.05 - new (copied from bottle-ft-v2)
 *
 * (c) 2019,2020 fnh, hendrich@informatik.uni-hamburg.de
 */
 
 // see end of file for our #include's...

eps = 0.01;
fn  = 250;

fdm_fudge = 0.2; // extra size for bores/holes/nuts due to shrinkage

explode_distance = 20; // 29; // 0 10 20 50;
arduino_explode_distance = 25; // 15;

pcb_thickness = 1.0;



// select components and options.
// Enable one or more of these before exporting to STL:
//

make_base_ring      = true;              // bottom + spiral parts of the FT
base_ring_inbus_instead_of_grub_screws = true;

make_fin_ring       = true;
make_spare_fins     = false;

make_upper_ring     = true;              // upper (optical sensor mounting) part
make_central_arduino_plate = true;
make_arduino_cutout = true;
show_sensors        = true;

make_arduino_ring   = false;
show_arduino_nano   = true;

make_distance_ring  = false;




// enable/disable optional components, check before exporting to STL:
//
use_colors = true;
h_inner = 18;
h_outer = 15;
h_fin_ring = 1;

show_sensor_blocks    = false;
cutout_sensor_blocks  = false;
show_sensor_carriers  = false;
show_arduino_pro_mini = false;


// main dimensions
//

M2NUTs = 4;    // diameter across sides
M2NUTe = 4.38; // outer diameter across edges
M2NUTm = 1.6;  // nut height
M2BORE = 2.0;
M2SCREWdk = 3.8; // inbus screw head diameter
M2SCREWk  = 2.0; // inbus screw head height

M25NUTs = 5;
M25NUTe = 5.45;
M25NUTm = 2;
M25BORE = 2.5;
M25SCREWdk = 4.5; // inbus screw head diameter
M25SCREWk  = 2.5; // inbus screw head height

M3NUTs = 5.5;
M3NUTe = 6.08;
M3NUTm = 2.4;
M3BORE = 3.0;   // add fudge / redefine on command line as neede
M3SCREWdk = 5.5; // inbus screw head diameter
M3SCREWk  = 3.0; // inbus screw head height

M4NUTs = 7.0;
M4NUTe = 7.8;
M4NUTm = 3.2;
M4BORE = 4.0;
M4SCREWdk = 7.0; // inbus screw head diameter
M4SCREWk  = 4.0; // inbus screw head height

M5NUTs = 8.0;
M5NUTe = 8.63;
M5NUTm = 4.0;
M5BORE = 5.0;
M5SCREWdk = 8.5; // inbus screw head diameter
M5SCREWk  = 5.0; // inbus screw head height



 
 
//intersection() 
//{
// translate( [0,0,-0] ) 
//   cube( size=[110.0,140,50], center=true ); // 90,140,50
//  
//// translate( [0,0,0] ) cube( size=[90.0,140,50], center=true ); // 90,140,50
{
union() { 


if (make_base_ring) {
base_ring( 
  h_inner = h_inner,
  d_inner1 = 40,   // same as Sunrise M3207 & PA10-6C flange
  d_inner2 = 65,   // Sunrise M320745,135,225,315]
  inner_screw_radius = 28.2,
  // inner_screw_angles = [45,135,225,315]
  inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
  inner_screw_modes  = [0,1,0,1,0,1,0,2], // 0=bore, 1=bottom-head-cavity 2=bottom-hex-nut-cavity
  inner_screw_bore = M4BORE + 0.2,
  inner_screw_head_cavity_height = M4SCREWk + 0.5,
  inner_screw_head_cavity_diameter = M4SCREWdk + 0.2,
  inner_screw_hex_cavity_diameter = M4NUTe + 0.2,
  h_outer = h_outer,
  d_outer1 = 84,   // inner diameter of outer (moving) ring
  d_outer2 = 104,  // outer diameter of outer (moving) ring,
  outer_screw_radius = (84+104)/4, // (d_outer1 + d_outer2)/2,
  outer_screw_angles = [45,135,225,315],
  // v1 grub_screw_angles = [-5,+5,80,100,170,190,260,280],
  grub_screw_angles = [-5,+5,85,95,175,185,265,275],
  grub_screw_bore = M2BORE,
  lever_fixing_screw_angles = [-30,30,60,120,150,210,240,300],
  spring_thickness = h_outer-1
  );
}
  

if (make_fin_ring) {
translate( [0,0,h_outer + explode_distance + eps] ) 
fin_ring(
  h_fin_ring = 1.5, // h_fin_ring,
  d_outer1 = 84,   // inner diameter of outer (moving) ring
  d_outer2 = 104,  // outer diameter of outer (moving) ring,
  outer_screw_radius = (84+104)/4, // (d_outer1 + d_outer2)/2,
  outer_screw_angles = [45,135,225,315],
  // grub_screw_angles = [-7,+7,80,100,172,188,260,280],
  grub_screw_angles = [-7,+7,83,97,173,187,263,277],
  grub_screw_bore = M2BORE,
  lever_fixing_screw_angles = [-30,30,60,120,150,210,240,300],

  fin_radius = 90/2-1.5, // was: 90/2
  fin_angles = [-1,+1,89,91,179,181,269,271],
  fin_orientations = [180,0,180,0,180,0,180,0],
  fin_base_height = 1,
  fin_thickness = 1,
  fin_height = 4,       // was: 9
  fin_width = 6
  );
}



if (make_upper_ring) {
translate( [0,0,h_inner + 2*explode_distance + 0.7*eps] )
upper_ring( 
  h_inner = 14,
  d_inner1 = 40,   // same as Sunrise M3207 & PA10-6C flange
  d_inner2 = 70,  // was 65,
  inner_screw_radius = 28.2,
  inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 292.5, 337.5],
  inner_screw_modes  = [1,2, 1,2, 1, 1,2], // 0=bore, 1=upper-cavity 2=upper-hex-cavity
  // inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
  // inner_screw_modes  = [1,2, 1,2, 1,2 ,1,2], // 0=bore, 1=upper-cavity 2=upper-hex-cavity
  inner_screw_bore = M4BORE + 0.2,
  inner_screw_head_cavity_height = M4SCREWk + 2.5,
  inner_screw_head_cavity_diameter = M4SCREWdk + 0.2,
  
  h_outer = h_outer,
  d_outer1 = 84,   // inner diameter of outer (moving) ring
  d_outer2 = 104,  // outer diameter of outer (moving) ring,

  outer_screw_radius = (84+104)/4, // (d_outer1 + d_outer2)/2,
  outer_screw_angles = [45,135,225,315],

  lever_fixing_screw_angles = [-30,30,60,120,150,210,240,300],
  spring_thickness = h_outer-1,

  cable_channels = [45, 135, 222, 315],
  h_cable_channels = 4,

  arduino_angle = 60,  
  h_arduino_base_plate = 1
  );
  
  
  if (h_inner < (h_outer + h_fin_ring)) {
    echo( "ERROR: h_inner < (h_outer + h_fin_ring)", h_inner, h_outer, h_fin_ring ); 
  }
}


if (make_arduino_ring) { 
translate( [0,0,h_inner+13 + 3*explode_distance + 0.7*eps] )
arduino_ring(
  h_inner  = 8,
  d_inner1 = 40,   // same as Sunrise M3207 & PA10-6C flange
  d_inner2 = 70,  // was 65,
  inner_screw_radius = 28.2,
  inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
  inner_screw_modes  = [1,2, 1,2, 1,2 ,1,2], // 0=bore, 1=upper-cavity 2=upper-hex-cavity
  inner_screw_bore = M4BORE + 0.2,
  inner_screw_head_cavity_height = M4SCREWk + 0.5,
  inner_screw_head_cavity_diameter = M4SCREWdk + 0.2
);
}


if (make_distance_ring) {
h_distance_ring = 2.0;
 translate( [0,0,2*h_inner + 3.3*explode_distance + 0.5*eps] )
// translate( [0,0,h_inner + 0.9*explode_distance + 0.5*eps] )
distance_ring( 
  h = h_distance_ring,
  d_inner1 = 40,   // same as Sunrise M3207 & PA10-6C flange
  d_inner2 = 70,  // was 65,
  inner_screw_radius = 28.2,
  inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
  inner_screw_bore = M4BORE + 0.2
);
}

if (make_spare_fins) { spare_sensor_fins(); } 



}} // union; intersection









/**
 * the adjustable-fins ring to be screwed/glued on top of the base ring.
 * Fin height is originally as specified, but can then be raised a bit
 * turning the grub-screws inserted into the base ring.
 * 
 * Most parameters are expected to be the same as for base ring,
 * but fin height is of course the most important parameter here.
 */
module fin_ring(
  h_fin_ring = 1,
  d_outer1 = 95,   // inner diameter of outer (moving) ring
  d_outer2 = 120,  // outer diameter of outer (moving) ring,
  outer_screw_radius = (95+120)/4, // (d_outer1 + d_outer2)/2/2
  outer_screw_angles = [45,135,225,315],
  grub_screw_angles = [-5,+5,80,100,170,190,260,280],
  lever_fixing_screw_angles = [-30,30,60,120,150,210,240,300],
  lever_fixing_screw_bore = M25BORE,
  fin_angles = [-5,+5,80,100,170,190,260,280],
  fin_radius = (95+120)/4,
  fin_thickness = 1,
  fin_width = 3,
  fin_crop_height = 8,
  fin_base_height = 1, 
  fin_orientations = [180,0,180,0,180,0,180,0]
)
{
  color( "lightblue" )
  difference() {
    ring( d_inner=d_outer1, d_outer=d_outer2, h=h_fin_ring, center=false, fn=200 );

    translate( [0,0,h_fin_ring/2] ) 
      cube( size=[d_outer2+eps,h_fin_ring+eps,h_fin_ring+eps], center=true );
    translate( [0,0,h_fin_ring/2] ) 
      cube( size=[h_fin_ring+eps,d_outer2+eps,h_fin_ring+eps], center=true );

    // M3 screw bores
    for( theta=outer_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [outer_screw_radius, 0, -eps/2] ) 
          rotate( [0,0,30] )
            cylinder( d=M3NUTe, h=h_fin_ring+eps, center=false, $fn=6 );
    }
    
    // bores to fix fin-ring on base-ring
    for( theta=lever_fixing_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [outer_screw_radius, 0, -eps/2] ) 
          cylinder( d=lever_fixing_screw_bore, h=h_fin_ring+eps, center=false, $fn=50 );
    }
  }  

  fin_v1 = false;
  fin_v2 = true;

  if (fin_v1) {
  // sensor fins are directly on-top of the grub screws,
  // oriented in tangential direction (override as needed)
  f = h_fin/2;
  for( i=[0,1,2,3,4,5,6,7]  ) {
    theta = grub_screw_angles[i];
    sign  = fin_orientations[i];
    rotate( [0,0,theta] ) 
      translate( [fin_radius, 0, h_fin_ring-eps/2] ) 
        rotate( [90*sign-90,-90,0] ) {
          // cylinder( d=1, h=h_outer+eps, center=false, $fn=50 ); 
          linear_extrude( height=fin_thickness, center=true ) {
            polygon( [[0,-f], [2*f,f], [0,f]] ); 
          }
        }
    }
  }

  if (fin_v2) {  
  // sensor fins are directly on-top of the grub screws,
  // oriented in tangential direction (override as needed)
  // fin_angles = [-1,1,30,31,32,33,34,35,36,37];
  bh = fin_base_height; // stem base height
  hw = fin_width; // 45-deg triangle height  
  ch = fin_height; // crop height
  hh = min( ch, bh+hw );
  dy = max( 0, hw-ch );
  for( i=[0,1,2,3,4,5,6,7]  ) {
    theta = fin_angles[i];
    sign  = fin_orientation[i];
    rotate( [0,0,theta] ) 
      translate( [fin_radius, 0, h_fin_ring-eps/2] ) {
//      cylinder( d=1, h=50, center=false, $fn=10 );
        #rotate( [fin_orientations[i], 270, 0] ) 
        linear_extrude( height=fin_thickness, center=true ) {
          polygon( [[0,0], [bh+hh,0], [bh+hh,dy], [bh,hw], [0,hw]] ); 
        }
      }
    }
  }
  
}


/* ########################################################### */
debug_extrude = false;
if (debug_extrude) {
translate(  [100,0,0] ) {
  bh = 3; // stem base height
  hw = 10; // 45-deg triangle width
  ch = 6; // triangle crop height
  hh = min( ch, bh+hw );
  dy = max( 0, hw-ch );
  cylinder( d=1, h=20, center=false, $fn=10 );
  // rotate( [-90,270,0] ) 
  linear_extrude( height=5, center=true ) {
    polygon( [[0,0], [bh+hh,0], [bh+hh,dy], [bh,hw], [0,hw]] ); 
  }
  color( "salmon" )
  linear_extrude( height=1, center=true ) {
    polygon( [[0,0], [bh+hw,0], [bh,hw], [0,hw]] ); 
  }
}
}


/**
 * the bottom part of the F/T sensor, consisting of:
 * - the inner ring touching/holding the bottle
 * - the elastic spring connecting inner and outer rings
 * - the outer ring for grasping the sensofalser, with "fins"
 */
module base_ring(
  h_inner  = 15,
  d_inner1 = 50,   // inner diameter of inner (fixed) ring, can be zero
  d_inner2 = 76,   // outer diameter of inner (fixed) ring
  h_outer  = 13,
  d_outer1 = 95,   // inner diameter of outer (moving) ring
  d_outer2 = 120,  // outer diameter of outer (moving) ring,
  outer_screw_radius = (95+120)/4, // (d_outer1 + d_outer2)/2,
  inner_screw_bore = M3BORE,
  inner_screw_head_cavity_height = M3SCREWk + 1,
  inner_screw_head_cavity_diameter = M3SCREWdk + 0.5,
  inner_screw_radius = 32,
  inner_screw_angles = [45,135,225,315],
  outer_screw_angles = [45,135,225,315],
  grub_screw_angles = [-10,+10,80,100,170,190,260,280],
  grub_screw_bore = M2BORE,
  lever_fixing_screw_angles = [-30,30,60,120,150,210,240,300],
  spring_thickness = 13,
) 
{
  
  // the inner (fixed) ring
  gray04() 
  difference() {
    ring( d_inner=d_inner1, d_outer=d_inner2, h=h_inner, center=false, fn=300 );

    // M3 screw bores
    for( theta=inner_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [inner_screw_radius, 0, -eps/2] ) 
          cylinder( d=inner_screw_bore, h=h_inner+eps, center=false, $fn=300 );
    }

    // optional bottom M3 screw head cavities 
    if (inner_screw_head_cavity_height > 0) {
    for( theta=inner_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [inner_screw_radius, 0, -eps/2] ) 
          cylinder( d=inner_screw_head_cavity_diameter, 
                    h=inner_screw_head_cavity_height, center=false, $fn=50 );
    }}
  }  

  
// outer (moving) ring with screw bores, hex-nut cutouts, and axis markers
  gray08()
  translate( [0,0,0.2*eps] ) 
  difference() { 
    // main outer ring
    ring( d_inner=d_outer1, d_outer=d_outer2, h=h_outer, center=false, fn=300 );

    // M3 screw bores
    for( theta=outer_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [outer_screw_radius, 0, -eps/2] ) 
          cylinder( d=M3BORE, h=h_outer+eps, center=false, $fn=50 );
    }

    // M3 hex nut cutouts
    for( theta=outer_screw_angles ) {
      h_M3NUT = M3NUTm + 1.0;
      rotate( [0,0,theta] ) 
        translate( [outer_screw_radius, 0, h_outer-h_M3NUT+eps] ) 
          rotate( [0,0,30] ) 
            cylinder( d=M3NUTe, h=h_M3NUT, center=false, $fn=6 );
    }

    // grub screw holes
    for( theta=grub_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [outer_screw_radius, 0, -eps/2] ) 
          cylinder( d=grub_screw_bore, h=h_outer+eps, center=false, $fn=50 );
    }
    
    // extra screw-head cutouts if using normal screws instead of grub screws
    h_head = max( M3SCREWk, h_outer-5 );
    if (base_ring_inbus_instead_of_grub_screws) {
      for( theta=grub_screw_angles ) {
        rotate( [0,0,theta] ) 
          translate( [outer_screw_radius, 0, -eps/2] ) 
            cylinder( d=M3SCREWdk, h=h_head, center=false, $fn=50 );
      }      
    }
  
    // adjustable lever ring bores
    for( theta=lever_fixing_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [outer_screw_radius, 0, -eps/2] ) 
          cylinder( d=M2BORE, h=h_outer+eps, center=false, $fn=50 );
    }
    
    // small cuts as axis markers
    for( theta=[0,90,180,270] ) {
      rotate( [0,0,theta] ) 
        translate( [d_outer2/2,0,-eps/2] )
          cylinder( d=(theta==0 ? 3 : 1.5), h=h_outer+eps, center=false, $fn=4 );
    }
  } // difference (outer ring) 

  // spiral spring starts here
  //
  ix = 3.8; iy = 9;  iz = spring_thickness;  // inner spring mounting block
  ax = 4.9; ay = 8;  az = spring_thickness;  // outer spring mounting block
  
  // inner spring mounting blocks
  gray08()
  for( theta=[0,90,180,270] ) {
    rotate( [0,0,theta] ) 
      translate( [d_inner2/2+ix/2-1,0,az/2] )  
        rotate( [0,0,6] )
          cube( size=[ix,iy,iz], center = true );
  }

  // outer spring mounting blocks
  gray08()
  for( theta=[0,90,180,270] ) {
    rotate( [0,0,theta-45] ) 
      translate( [d_outer1/2-ax/2+1,0,az/2] ) 
        rotate( [0,0,2] )
          cube( size=[ax,ay,az], center = true );
  }

  // red06()
  color( "salmon" ) 
  for( theta=[0,90,180,270] ) {
    rotate( [0,0,theta] ) 
      translate( [1,-2.7,0.3*eps] ) 
        rotate( [0,0,-4] ) 
          spring_segment( d_inner=d_inner2+6, d_outer=d_inner2+12, h=spring_thickness );
  }
} // end module base_ring


/*
##########################################################
##########################################################
######## Upper Ring ######################################
##########################################################
##########################################################
*/


/**
 * the upper part of the F/T sensor, consisting of:
 * and inner ring to be glued onto the base_ring,
 * mounting blocks for the optical sensor modules ("carriers"),
 * hex nut screw holes for the mounting screws
 * a carrier for the Arduino microcontroller
 */
module upper_ring(
  h_inner = 10,
  d_inner1 = 40,   // same as Sunrise M3207 & PA10-6C flange
  d_inner2 = 65,   // Sunrise M320745,135,225,315]
  inner_screw_radius = 28.2,
  inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
  inner_screw_modes = [0,1,0,1,0,1,0,2], // 0=bore, 1=upper-screw-head, 2=upper-hex-nut-cavity
  inner_screw_bore = M4BORE + 0.2,
  inner_screw_head_cavity_height = M4SCREWk + 0.5,
  inner_screw_head_cavity_diameter = M4SCREWdk + 0.2,
  inner_screw_hex_cavity_diameter = M4NUTe + 0.2,
  
  h_interconnection_ring = 3,
  h_outer = 10,
  d_outer1 = 84,   // inner diameter of outer (moving) ring
  d_outer2 = 104,  // outer diameter of outer (moving) ring,
  lever_fixing_screw_angles = [-30,30,60,120,150,210,240,300],
  cable_channels = [45, 135, 225, 315],
  h_cable_channels = 3,

  h_arduino_base_plate = 2,
  arduino_angle = 22.5+45-5,
)
{
  // color( "lightgreen" )
  difference() { // inner ring with screw bords
    union() {
      // thick main inner ring 
      // 
      color( "lightgreen" )
      ring( d_inner=d_inner1, d_outer=d_inner2, h=h_inner, center=false, fn=200 );
      
      // four 2DOF_sensor carriers
      xxx = 43.5;
      hxx = 10;
      for( theta=[0,90,180,270] ) {
        rotate( [0,0,theta] )
          translate( [xxx,0,hxx] ) rotate( [0,180,0] ) 
            2DOF_sensor_carrier( show_sensor=show_sensors ); 
      }

      // thin "interconnection" ring (with sensor carrier cutouts)
      color( "lightgreen" )
      difference() {
        ring( d_inner=d_inner2, d_outer=d_outer2, h=h_interconnection_ring, center=false, fn=200 );
        for( theta=[0,90,180,270] ) {
          rotate( [0,0,theta] )
            translate( [xxx-eps,0,hxx-eps] ) rotate( [0,180,0] ) 
              2DOF_sensor_carrier_box();
        }
      }

      // thin outer "rim"
      color( "lightgreen" )
      ring( d_inner=d_outer2-2, d_outer=d_outer2, h=h_inner, center=false, fn=200 );

      // optional arduino mounting plate
      color( "lightgreen" )
      if (make_central_arduino_plate)
        rotate( [0,0,arduino_angle] ) 
          translate( [0,0,h_arduino_base_plate/2] ) 
            cube( size=[60,20,h_arduino_base_plate], center=true );
      
    } // union

    // M3 screw bores
    for( theta=inner_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [inner_screw_radius, 0, -eps/2] ) 
          cylinder( d=inner_screw_bore, h=h_inner+eps, center=false, $fn=300 );
    }

    // optional bottom M3 screw head cavities 
    if (inner_screw_head_cavity_height > 0) {
      for( i=[0:7] ) {
        theta = inner_screw_angles[i];
        mode  = inner_screw_modes[i];
        hh = (inner_screw_modes[i] == 1) ? inner_screw_head_cavity_height : 0;
        rotate( [0,0,theta] ) 
          translate( [inner_screw_radius, 0, h_inner - inner_screw_head_cavity_height + eps/2] ) 
            cylinder( d=inner_screw_head_cavity_diameter, 
                      h=hh, center=false, $fn=50 );
      }

      for( i=[0:7] ) {
        theta = inner_screw_angles[i];
        mode  = inner_screw_modes[i];
        dddd = (inner_screw_modes[i] == 2) ? inner_screw_hex_cavity_diameter : 0;
        rotate( [0,0,theta] ) 
          translate( [inner_screw_radius, 0, h_inner - inner_screw_head_cavity_height + eps/2] ) 
            cylinder( d=dddd,
                      h=inner_screw_head_cavity_height, center=false, $fn=6 );
      }
    }

    // axis alignment markers (notches) at +/-x and +/- y
    // 
    for( theta=[0,90,180,270] ) { 
      rotate( [0,0,theta] ) 
        translate( [d_outer2/2, 0, -eps] )
          cylinder( d = (theta == 0 ? 3 : 1.5), h=h_outer+50, center=false, $fn=10 );
    }
    
    // cable-channels, if any
    for( theta=cable_channels ) {
      lcc = (d_inner2 - d_inner1)/2 + eps;
      xcc = (d_inner2 + d_inner1)/4;
      rotate( [0,0,theta] )
        translate( [xcc, 0, h_inner-h_cable_channels/2+eps] )
          cube( size=[lcc,h_cable_channels, h_cable_channels], center=true ); 
    }

    // extra cutouts for the heads of the "lever fixing screws"
    // on the base plate    
    for( theta=lever_fixing_screw_angles ){
      rotate( [0,0,theta] ) 
        translate( [outer_screw_radius, 0, -eps] )
          cylinder( d=M4SCREWdk, h=h_interconnection_ring+1, center=false, $fn=100 );
    }

    if (make_arduino_cutout) {
      // main Arduino cutout
      nano_height = 7;
      rotate( [0,0,arduino_angle] ) 
        translate( [-2,0,h_arduino_base_plate + (h_inner-h_arduino_base_plate)/2+eps] ) 
          cube( size=[48, 20, (h_inner-h_arduino_base_plate)+eps], center=true );

      // Arduino usb cable cutout
      hccc = (h_inner - h_arduino_base_plate - 1);
      hcc = hccc > 9.0 ? 9.0 : hccc;
      rotate( [0,0,arduino_angle] ) 
        translate( [-48/2 -8/2, 0, h_arduino_base_plate+hcc/2+0.3*eps] )
          cube( size=[20, 13, hcc], center=true );
      
      // optional cable cutout in the outer rim: mini-USB 10x7
      hrcc = max( h_arduino_base_plate, h_interconnection_ring ) + 8/2;
      rotate( [0,0,arduino_angle] ) 
        translate( [-d_outer2/2, 0, hrcc] ) 
            cube( size=[6,12,8.5], center=true );

    }
    
  } // difference 

  if (show_arduino_nano) {
    translate( [0,0,h_arduino_base_plate + arduino_explode_distance ] ) rotate( [0,0,arduino_angle] )
      arduino_nano();
  }
  
} // upper_ring



/*
##########################################################
##########################################################
##########################################################
##########################################################
##########################################################
*/


/**
 * a separate ring to mount/carry an embedded Arduino (nano)
 * or Teensy microcontroller.
 * hex nut screw holes for the mounting screws
 */
module arduino_ring(
  h_inner  = 15,
  d_inner1 = 40,   // same as Sunrise M3207 & PA10-6C flange
  d_inner2 = 65,   // Sunrise M320745,135,225,315]
  inner_screw_radius = 28.2,
  inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
  inner_screw_modes = [0,1,0,1,0,1,0,2], // 0=bore, 1=upper-screw-head, 2=upper-hex-nut-cavity
  inner_screw_bore = M4BORE + 0.2,
  inner_screw_head_cavity_height = M4SCREWk + 0.5,
  inner_screw_head_cavity_diameter = M4SCREWdk + 0.2,
  inner_screw_hex_cavity_diameter = M4NUTe + 0.2,
  cable_channels = [45, 135, 225, 315],
  h_cable_channels = 3,
  h_arduino_base_plate = 1,
  arduino_angle = 22.5+45-5,
)
{
  // color( "lightgreen" )
  difference() { // inner ring with screw bords
    union() {
      // thick main inner ring 
      // 
      ring( d_inner=d_inner1, d_outer=d_inner2, h=h_inner, center=false, fn=200 );

      if (h_arduino_base_plate > 0) {
        translate( [0,0,h_arduino_base_plate/2] ) rotate( [0,0,arduino_angle+90] )
          cube( size=[20,60,h_arduino_base_plate], center=true );
      }
    } // union

    // M3 screw bores
    for( theta=inner_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [inner_screw_radius, 0, -eps/2] ) 
          cylinder( d=inner_screw_bore, h=h_inner+eps, center=false, $fn=300 );
    }
    
    // main Arduino cutout
    nano_height = 7;
    rotate( [0,0,arduino_angle] ) 
      translate( [-2,0,h_arduino_base_plate + (h_inner-h_arduino_base_plate)/2+eps] ) 
        cube( size=[48, 20, (h_inner-h_arduino_base_plate)+eps], center=true );

    // Arduino usb cable cutout
    hcc = h_inner - h_arduino_base_plate - 1;
    rotate( [0,0,arduino_angle] ) 
      translate( [-48/2 -10/2, 0, h_arduino_base_plate+hcc/2+0.3*eps] )
        cube( size=[20, 16, hcc], center=true );

    // optional bottom M3 screw head cavities 
    if (inner_screw_head_cavity_height > 0) {
      for( i=[0:7] ) {
        theta = inner_screw_angles[i];
        mode  = inner_screw_modes[i];
        hh = (inner_screw_modes[i] == 1) ? inner_screw_head_cavity_height : 0;
        rotate( [0,0,theta] ) 
          translate( [inner_screw_radius, 0, h_inner - inner_screw_head_cavity_height + eps/2] ) 
            cylinder( d=inner_screw_head_cavity_diameter, 
                      h=hh, center=false, $fn=50 );
      }

      for( i=[0:7] ) {
        theta = inner_screw_angles[i];
        mode  = inner_screw_modes[i];
        dddd = (inner_screw_modes[i] == 2) ? inner_screw_hex_cavity_diameter : 0;
        rotate( [0,0,theta] ) 
          translate( [inner_screw_radius, 0, h_inner - inner_screw_head_cavity_height + eps/2] ) 
            cylinder( d=dddd,
                      h=inner_screw_head_cavity_height, center=false, $fn=6 );
      }
    }

    // axis alignment markers (notches) at +/-x and +/- y
    // 
    for( theta=[0,90,180,270] ) { 
      rotate( [0,0,theta] ) 
        translate( [d_outer2/2, 0, -eps] )
          cylinder( d = (theta == 0 ? 3 : 1.5), h=h_outer+50, center=false, $fn=10 );
    }
  }
  
  if (show_arduino_nano) {
    translate( [0,0,h_arduino_base_plate] ) rotate( [0,0,arduino_angle] )
      arduino_nano();
  }
} // end arduino_ring


module distance_ring(
  h = 1.0,
  d_inner1 = 40,   // same as Sunrise M3207 & PA10-6C flange
  d_inner2 = 65,   // Sunrise M320745,135,225,315]
  inner_screw_radius = 28.2,
  inner_screw_angles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5],
  inner_screw_bore = M4BORE + 0.2,
)
{
  color( "lightgreen" )
  difference() { // inner ring with screw bords
    ring( d_inner=d_inner1, d_outer=d_inner2, h=h, center=false, fn=200 );

    // M3 screw bores
    for( theta=inner_screw_angles ) {
      rotate( [0,0,theta] ) 
        translate( [inner_screw_radius, 0, -eps/2] ) 
          cylinder( d=inner_screw_bore, h=h_inner+eps, center=false, $fn=300 );
    }
  }
}




/*
##########################################################
##########################################################
##########################################################
##########################################################
##########################################################
*/



module spring_segment( d_inner=85, d_outer=90, h=10, fn=fn )
{
  intersection() {
    ring( d_inner=d_inner, d_outer=d_outer, h=h, center=false, fn=fn );

    union() {  
      cube( size=[130,130+eps,h+eps], center=false );
      rotate( [0,0,-40] ) 
        cube( size=[130,130+eps,h+eps], center=false );
    }
  }
}



/**
 * combined 2-axis sensor fin, aligned along x-axis, 
 * block z-axis interrupter from below and x-axis interruptor 
 * from the inside.
 * origin centered in x and y and at bottom z=0.
 * Total height h to block the z-axis interruptor,
 * total width w to block the x-axis interruptor, extra height hh,
 * extra x width ww.
 * Thickness (along y) t.
 */
module sensor_fin_v2( t=1, w=1.5*2.54, ww=2.0, h=5.8, hh=1.5 )
{
  translate( [-w/2,0,h/2+hh/2] ) 
    cube( size=[w, t, h+hh], center=true );
  translate( [w/2+ww/2,0,h/2] ) 
    cube( size=[w+ww, t, h], center=true );
}


module sensor_fin_v3( t=1, w=1.5*2.54, ww=2.0, h=6.5, hh=1.5 )
{
  translate( [-w/2,0,h/2+hh/2] ) 
    cube( size=[w, t, h+hh], center=true );
  translate( [w/2+ww/2,0,h/2] ) 
    cube( size=[w+ww, t, h], center=true );
}


module sensor_fin() {
  // sensor_fin_v2();
  sensor_fin_v3();
}



/**
 * sensors fins of original size mounted on a thin plate.
 * To be used for repairs, in case the original fins got damaged
 * or broken.
 */
module spare_sensor_fins() {
  
  for( theta=[0,90,180,270] ) { // four sensor fins
    rotate( [0,0,theta] ) 
      difference() {
        translate( [0, 20-10/2+1/2, 1/2] ) 
          cube( size=[15,10,1], center=true );
        
        translate( [0, 20-10+1/2, 0.5] ) 
          cylinder( d=1, h=1+eps, center=true, $fn=20 );
      }
    
    rotate( [0,0,theta] ) 
      translate( [0,-20,0] ) 
        sensor_fin();
  }
}


module 2DOF_sensor_carrier_box(
  xx = 15, yy = 17, hh = 10,
)
{
    union() {
      // main box around both sensors
      translate( [0,0,hh/2] ) 
        cube( size=[xx,yy,hh-eps], center=true );
    
      // extra box to give more "slot length"
      translate( [0,0,hh/2] ) 
        cube( size=[8,26,hh-eps], center=true );
    }
}    


 
/**
 * a small adjustable (!) carrier for two Vishay TCST1103
 * optocouplers mounted on a breaboard PCB. The sensor slots
 * are aligned along the y-axis.
 * Sensor dimensions are bx 11.0, by 6.3, bz 3.1 mm.
 *
 * To ensure printability on FDM printers, we use slight
 * "corner cutouts". This should help against material shrinkage
 * during cooling of the printed part, and will allow us to
 * "file" the carrier a bit for a snug fit of the sensors.
 * The middle wall can be broken away if needed.
 */
module 2DOF_sensor_carrier( 
  xx = 15, yy = 17, hh =10,  // main carrier block (v2 had hh=10)
  show_sensor=true 
)
{
  bx = 11.9;
  by = 6.3;
  bz = 3.1;
  // baseplate with cutouts for the sensors
  //
  
  dd = 0.6; // corner cutout cylinder diameter
color( [1.0,0.4,0.1] ) {
  difference() {
    2DOF_sensor_carrier_box();
    
    // central slot cutout, slot width 3.1, we use 6.2 here
    translate( [0,0,(hh)/2 ] )
      cube( size=[6.2, 7*2.54 + 6.3, hh+2*eps], center=true );
    
    // subtract the sensor module itself
    translate( [-1.5*2.54,-3*2.54,-pcb_thickness-eps] )  
      2DOF_sensor_module( show_optical_axis=false );
    
    // corner cutouts
    for( x=[-bx/2, bx/2] ) {
      for( y=[ -1.5*2.54-by/2, -1.5*2.54+by/2, 1.5*2.54-by/2, 1.5*2.54+by/2] ) {
        translate( [x,y,-eps] ) 
          cylinder( d=dd, h=hh+2*eps, center=false, $fn=20 );
      }
    }    
  }
  
}
  
  if (show_sensor) {
    translate( [-1.5*2.54,-3*2.54,-pcb_thickness-explode_distance] )  2DOF_sensor_module();
  }
}




module 2DOF_sensor_carrier_mounting_block(
  xx = 15, yy = 16, hh = 10,  // main carrier block
  xx2 = 8.0, yy2 = 8.0,       // side mounting screw blocks
  extra_y = 1.0, // make outer walls a bit thicker
)
{
  dyy = yy/2 + yy2/2;

  difference() {
    translate( [-xx/2-xx2/2, 0, hh/2] ) 
      cube( size=[xx2, yy+2*yy2-2+2*extra_y, hh-2], center=true );

    // hex nut cutout 
    m3nut = M25NUTe + fdm_fudge;
    for( dy = [-dyy, +dyy] ) {
      translate( [-xx/2+1, dy, hh/2] ) 
          rotate( [0,270,0] ) 
            cylinder( d=m3nut, h=xx2+2+5*eps, center=false, $fn=6 );
    }
  }
  
  
  for( dy = [-dyy, +dyy] ) {
    translate( [-xx/4, dy, hh/2] ) 
    difference() {
      // main body
      translate( [0,extra_y/2*sign(dy),0] )
      cube( size=[xx/2, yy2-2+extra_y, hh-2], center=true );
    
      // screw bore through the block
      m3bore = 3.0 + fdm_fudge;
      translate( [0, 0, 0] )
        rotate( [0,90,0] ) 
          cylinder( d=m3bore, h=xx/2+eps, center=true, $fn=20 );

      // hex nut cutout 
      // translate( [-xx/4, 0, 0] )
      //   rotate( [0,90,0] ) 
      //     cylinder( d=5.5, h=5.0, center=true, $fn=6 );
    } // difference
  } // for
  
}
 
   





/** 
 * a small (4x8 holes) breadboard to carry two Vishay TCST1103 
 * optocouplers. The slots are aligned along the y-axis.
 * When mechanically inserted into the sensor base plate, 
 * this serves to measure y- and z-deflection of one elastic arm.
 * Note: The origin of this assembly is at the bottom-left hole (bore)
 * of the breadboard. 
 * Cables are expected to be soldered from the bottom, or into
 * the "extra" four holes at ny=7.
 */
module 2DOF_sensor_module( show_optical_axis=true ) {
    dx1 =  1.5 * 2.54; // x and y offsets of the optocouplers
    dy1 =  1.5 * 2.54;    
    dy2 =  4.5 * 2.54;    

    translate( [0,1*2.54,0] ) breadboard( nx=4, ny=5, z=pcb_thickness, bore=1.0, fn=15, epoxy=false );
    translate( [dx1, dy1, pcb_thickness] ) rotate( [0,0,180] ) vishay_TCST1103( show_optical_axis=show_optical_axis, wire_length=2 );
    translate( [dx1, dy2, pcb_thickness] ) rotate( [0,0,0] ) vishay_TCST1103( show_optical_axis=show_optical_axis, wire_length=2 );
}    
    
    





module ring( d_outer, d_inner, h, fn=fn, center ) {
  difference() {
    cylinder( d=d_outer, h=h, $fn=fn, center=center );
    translate( [0,0,-eps] ) cylinder( d=d_inner, h=h+2*eps, $fn=fn, center=center );

  }
}



module gray04() {
  if (use_colors) color( [0.3,0.3,0.3] ) children();
  else children();
}

module gray05() {
  if (use_colors) color( [0.5,0.5,0.5] ) children();
  else children();
}

module gray06() {
  if (use_colors) color( [0.7,0.7,0.7] ) children();
  else children();
}

module gray08() {
  if (use_colors) color( [0.8,0.8,0.8] ) children();
  else children();
}

module red06() {
  if (use_colors) color( [0.8,0.3,0.3] ) children();
  else children();
}
    
    
include <breadboard.scad>
include <optokoppler.scad>
include <electronics.scad>
//include <ati-nano17e.scad>
