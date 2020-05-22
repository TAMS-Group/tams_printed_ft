/** six-axis-thin.scad
 * 
 * Parametric 3D-printed 6-axis "thin pipe" F/T sensor based on 
 * optical IR reflex sensors (Everlight 9904).
 * The design is intended to have a diameter as small as possible,
 * without particular regard for sensor height.
 * 
 * 2020.01.18 - add "coin calibration rig"
 *
 * 2019.01.29 - moveable sensors and distance-plate
 * 2019.01.29 - better spring without Sollbruchstelle
 *
 * 2018.08.06 - try 9904-type sensors
 * 2018.01.04 - add spacemouse design
 * 2017.12.29 - new
 * 2017.10.16 - based on x1 new layout for "fork type" sensors
 * 
 * (C) 2017, 2018, fnh, hendrich@informatik.uni-hamburg.de
 */
 
 


pcb_thickness = 0.8;
fn = 200;
eps = 0.001;
explode_distance = 10; // extra "explosion" z-distance of parts 
alpha = 0.89; // transparency of outer ring
fdm_fudge = 0.4; // extra diameter of holes when using FDM printing...


M2NUTs = 4;    // diameter across sides
M2NUTe = 4.38; // outer diameter across edges
M2NUTm = 1.6;  // nut height

M25NUTs = 5;
M25NUTe = 5.45;
M25NUTm = 2;

M3NUTs = 5.5;
M3NUTe = 6.08;
M3NUTm = 2.4;

M4NUTs = 7.0;
M4NUTe = 7.8;
M4NUTm = 3.2;

M5NUTs = 8.0;
M5NUTe = 8.63;
M5NUTm = 4.0;

M2INBUSs = 3.8; // inbus head diameter
M2INBUSm = 2.0; // inbus head height

M3INBUSs = 5.5; // inbus head diameter
M3INBUSm = 3.0; // inbus head height

M4INBUSs = 7.0; // inbus head diameter
M4INBUSm = 4.0; // inbus head height

M5INBUSs = 8.4; 
M5INBUSm = 4.9;


// translate( [0,0,-50] ) pa10_drill_chunk_adapter();
// translate( [0,0,50] ) coin_calibration_rig();
// translate( [0,0,0] ) coin_calibration_stops();
// translate( [0,0,0] ) coin_calibration_piston();


// translate( [0,0,0] )  spacemouse( make_teensy=true ); // everything
// translate( [0,0,50] ) spacemouse_nano17_adapter( ati=false );


translate( [0,0,0] ) spacemouse( 
  make_groundplate=true,
  make_teensy = true,
  make_baseplate = true,
  show_sensors = true,
  make_elastic = true,
  make_sensor_fins = false, // don't want them on 9904 design
  make_cover = false,
  
  make_distance_calibration_plate = false,
  dcp_h1 = 2.3, dcp_h2 = 3.3,
  // dcp_h1 = 2.5, dcp_h2 = 3.8,
  make_distance_calibration_plate2 = false
);


//for( h=[1.0, 1.5, 2.0, 2.5, 3.0, 3.5] ) {
//  translate( [0,h*35,h] ) rotate( [0,180,0] ) 
//    spacemouse(
//    make_groundplate = false, 
//    make_baseplate = false,
//    show_sensors = false,
//    make_teensy = false,
//    make_elastic = false,
//    make_cover = false,
//    make_distance_calibration_plate = false,
//    make_sensor_fins = false,
//    make_central_pillar_spacing_plate = true,
//    h_central_pillar_spacing_plate = h
//  );
//}


//translate( [0,0,0] ) spacemouse( 
//    make_groundplate = false, 
//    make_baseplate = false,
//    show_sensors = false,
//    make_teensy = false,
//    make_elastic = false,
//    make_cover = false,
//    make_distance_calibration_plate = false,
//    make_sensor_fins = false,
//    make_central_pillar_spacing_plate = true,
//    h_central_pillar_spacing_plate = 1.5
//);



//translate( [0,0,0] ) spacemouse( 
//  make_groundplate=false,
//  show_sensors = false,
//  make_teensy = false,
//  make_baseplate = false,
//  show_rim = true,
//  make_elastic = true,
//  make_cover = false
//);



//translate( [90,0,0] )   vishay_TCST1103();
//translate( [90,90,0] )  sensor_ring_radial();
//translate( [0,90,0] )   sensor_ring_tangential();
//translate( [-90,0,0] )  sensor_ring_mixed();





/* 
 * six-sensor ITR9904 version of a "space-mouse" 
 * (6-DOF f/t sensor) with a couple of buttons
 * and integrated Arduino nano / Teensy 3.2
 * microcontroller.
 * 
 * The design is based on a radial layout of the
 * six sensors, three of which are used to detect
 * z-axis deflection (for Fz, Tx, Ty), with the
 * remaining three to detect horizontal deflection
 * (for Fx, Fy and Tz). 
 *
 */
module spacemouse(
  explode_distance = explode_distance, // use 0 to test assembly
  show_sensors = true,
  make_groundplate = true, // curerently, with embedded Teensy 3.2
  make_teensy = true,
  
  make_baseplate = true,  
  make_baseplate_sensor_guidings = true,
  make_elastic = true,
  show_rim = true,
  make_cover = true,
  make_buttonplate = false,
  make_arduinoplate = false,
  make_central_pillar_spacing_plate = false,

  groundplate_thickness = 5.0,
  groundplate_diameter = 42.0,
  groundplate_separation = 8.0,
  groundplate_hex_pillar = false, // disable when embedding the Teensy
  groundplate_outer_screw_position = 17, // this is a radius!
  groundplate_outer_pillar_diameter = 7, // should be > screw_bore
  groundplate_outer_screw_bore = 3.1 + fdm_fudge, // M3 + eps
  groundplate_outer_screw_head_diameter = 6.0 + fdm_fudge, // M3 inbus screw head + eps
  groundplate_outer_screw_head_height = 3.5, // M3 inbus screw head + eps
  groundplate_outer_nut_diameter = 6.4 + fdm_fudge, // M3 hex nut
  groundplate_outer_nut_height = 2.5, // M3 hex nut
  groundplate_color = [0.7,0.7,0.7, 1],
  
  r_sensor    = 14.0, // radial offset of sensor centers
  baseplate_thickness = 3.0, // height of inner baseplate
  baseplate_diameter = 42.0, // diameter of inner (sensor) baseplate
  baseplate_color = [135/255, 206/255, 250/255, 1], // lightskyblue 135 206 250

  central_bore_diameter = 4.2 + fdm_fudge, // M4
  central_nut_diameter = 8.0 + fdm_fudge, // M4 hex nut outer diameter + eps
  central_nut_height = 3.5, // M4 hex nut height + eps
  central_screw_head_height = 4.2, // M4 inbus 
  central_screw_head_diameter = 7.4 + fdm_fudge, // M4 inbus 
  alignment_bar_diameter = 2.0,
  eta = 0.5, // optional offset of the alignment bars

  buttonplate_diameter=70,
  buttonplate_thickness=2.0,
  r_buttons=29, // radius from center of sensor

  elastic_rim_z_offset = 2, // 
  elastic_rim_thickness = 7.0, // v4: 15.0
  elastic_pillar_diameter = 14,
  elastic_rim_outer_diameter = 42.0, // same as baseplate
  elastic_rim_inner_diameter = 27.0, 
  elastic_rim_screw_bore = 2.6 + fdm_fudge, // M2.5 screw
  elastic_rim_nut_diameter = 5.8 + fdm_fudge, // M2.5 hex nut + eps
  elastic_rim_nut_height = 2.5, // M2.5 hex nut + eps
  elastic_arm_z_thickness = 7.0, // v4: 14.5 v3 17.5 v2: 7.5
  elastic_arm_x_thickness = 2.02, // v3: 0.7, // v2: 1.6   0.7 is two extrusion widths
  elastic_rim_color = [1, 215/255, 0], // gold 255 215   0
  // elastic_rim_color = [144/255, 238/255, 144/255], // lightgreen 144 238 144
  // elastic_rim_color = [205/255, 92/255, 92/255], // indian red 205  92  92
  elastic_rim_color = [218/255, 112/255, 214/255 ], // orchid
  elastic_rim_color = [106/255,  90/255, 205/255 ], // slate blue 106  90 205
  elastic_rim_color = [205/255, 133/255, 63/255], // peru 205 133  63
  
  x_tcst1103  = 11.9, // size of Vishay TCST1103 optocoupler
  y_tcst1103  = 6.3,
  z_tcst1103  = 8, // 10.8, // 10.8, // 6.0 + 2.0, // changed for ITR9904 real TCST1103 is 10.8, 
  z_tcst1103_beam = 8.1, // according to datasheet...
  
  x_itr9904  = 11.54, // from Everlight datasheet
  y_itr9904  =  4.2,
  z_itr9904  = 6.0,
  
  dcp_h1 = 1.0, // distance calibration plate
  dcp_h2 = 2.5,
  dcp_hex = true,
) 
{
  echo( "WARNING! WARNING!" );
  echo( "Remember to set z_tcst1103 to 10.8 (1103) or 8.0 (9904)!!!!!!!" );
  echo( "WARNING! WARNING!" );


  // six sensors in "radial" layout
  if (show_sensors) {
    for( theta= [0,60,120,180,240,300] ) {
        rotate( [0,0,theta] ) 
          translate( [r_sensor,0,-eps] ) 
            // scale a bit to avoid non-manifold issues when building the cavities later on
            scale( [1.001, 1.001, 1.001] ) 
              everlight_ITR9904( show_wires=true, wire_length=1.0 );
    }
  }
  
  // xxxtttxxx
  // distance-calibration-plate: we want the z-sensors
  // at a distance of about 1mm from the moving elastic plate, 
  // but the xy-sensors near the max of the sensor curve,
  // that is, approx 2.3mm for the ITR9904...
  //
  if (make_distance_calibration_plate) {
    // dcp_h1 = 1.0;
    // dcp_h2 = 2.3;
    translate( [0,0,50] ) 
    union() {
      ring( d_outer=2*(r_sensor+x_itr9904/3), 
            d_inner=2*(r_sensor-x_itr9904/3), $fn=fn, 
            h=dcp_h1 );
      difference() {  
        ring( d_outer=2*(r_sensor+x_itr9904/3), 
              d_inner=2*(r_sensor-x_itr9904/3), $fn=fn, 
              h=dcp_h2 );
        for( theta=[0,120,240] ) {
          rotate( [0,0,theta+30] )
            translate( [0,r_sensor,dcp_h2/2+eps ] )
              cube( size=[10, 20, dcp_h2+3*eps], center=true );
        }
      }
    }
  } // make_distance_calibration_plate


  // distance calibration plate with b/w sensors at maximum
  // sensitivity, distance sensors at greater distance,
  // using central hex-pillar cutout
  // 
  // dcp_h1 = 1;
  // dcp_h2 = 2;
  // 
  if (make_distance_calibration_plate2) {
    d_outer = 2*(r_sensor + x_itr9904/3) - 1;
    // d_hex   = (r_sensor - x_tcst1103/2);
    d_hex = 18;
    echo( "d_hex", d_hex );
    hh = dcp_h2 - dcp_h1;
    
    translate( [0,0,70] ) 
    difference() {
      cylinder( d=d_outer, h = dcp_h2, center=false, $fn=fn );
      rotate( [0,0,30] ) translate( [0,0,-eps] ) 
        cylinder( d= d_hex, 
                  h=dcp_h2+2*eps, $fn=6, center=false );
      
      for( theta=[0,120,240] )
        rotate( [0,0,theta] ) 
          translate( [d_outer/2, 0, dcp_h1 + hh/2] )
            cube( size=[d_outer, y_itr9904+0.5, hh+2*eps], center=true );
    }
  }


  if (make_groundplate) {
    hhh = groundplate_thickness + groundplate_separation;
    translate( [0,0, -hhh-explode_distance] )
    difference() {
      union() {
        // main groundplate cylinder
        color( groundplate_color )
        cylinder( d=groundplate_diameter, h=groundplate_thickness, $fn=fn, center=false );
        // hex pillar to the baseplate
        color( groundplate_color )
        if (groundplate_hex_pillar) {
          translate( [0,0,groundplate_thickness-eps] ) rotate( [0,0,30] ) 
            cylinder( r=r_sensor - x_tcst1103/2,
                      h=groundplate_separation+2*eps, $fn=6, center=false );
          // horizontal alignment bars on top of hex pillar
          translate( [0,eta,hhh] ) rotate( [0,90,0] )
            cylinder( d=alignment_bar_diameter, h=r_sensor, $fn=30, center=true );
          translate( [0,0,hhh] ) rotate( [0,90,90] )
            cylinder( d=alignment_bar_diameter, h=r_sensor, $fn=30, center=true );
        }
        // optional outer pillars
        color( groundplate_color ) 
        if (groundplate_outer_screw_position > 0) {
          pillar_angles = [0,60,120,180,240,300];  // without Teensy
          pillar_angles = [0,120,240]; // Teensy 2.0
          pillar_angles = [60,120,240,300]; // Teensy 3.2
          for( theta=pillar_angles ) {
            rotate( [0,0,theta+30] )
              translate( [groundplate_outer_screw_position,0,groundplate_thickness-eps] ) 
                cylinder( d=groundplate_outer_pillar_diameter, h=groundplate_separation+5*eps, $fn=fn, center=false );
          }
        }
        // embed Teensy 3.2 (16 bit A/D), outer size 14*2.54 / 7*2.54
        if (make_teensy == true) {
          rotate( [0,0,30] ) 
            translate( [0,0,groundplate_thickness+eps] ) teensy_32();
        }
        // Teensy holder
        color( groundplate_color ) 
        difference() {
          holder_height = 3;
          rotate( [0,0,30] ) 
            translate( [0,0,groundplate_thickness+holder_height/2-3*eps] )            
              cube( size=[37,20,holder_height], center=true ); 
          rotate( [0,0,30] ) 
            translate( [0,0,groundplate_thickness+holder_height/2-1.5*eps] )            
              cube( size=[14*2.54+0.2,7*2.54+0.2,holder_height+2*eps], center=true ); 
          rotate( [0,0,30] ) 
            translate( [0,0,groundplate_thickness+1.7*eps] ) 
              scale( [1.01, 1.01, 1] ) teensy_32( bores=false );
        }
      }
      
      // Teensy solder cutouts
      solder_height = 2;
      rotate( [0,0,30] ) {
        translate( [0,3*2.54,groundplate_thickness-solder_height/2+2*eps] )            
          cube( size=[37,3,solder_height], center=true );
        translate( [0,-3*2.54,groundplate_thickness-solder_height/2+2*eps] )            
          cube( size=[37,3,solder_height], center=true );
        translate( [6.5*2.54,0,groundplate_thickness-solder_height/2+2*eps] )            
          cube( size=[3,7*2.54,solder_height], center=true );
      }

      // central screw bore and nut cavity
      if (groundplate_hex_pillar) {
        translate( [0,0,-eps] )
          cylinder( d=central_bore_diameter, 
                    h=hhh+alignment_bar_diameter+3*eps, 
                    $fn=30, center=false );
        // central nut cavity
        translate( [0,0,-eps] )
          cylinder( d=central_nut_diameter, h=central_nut_height, $fn=6, center=false );
      }
       
      // six outer screw bores with nut cavities on top and bottom
      if (groundplate_outer_screw_position > 0) {
        // outer screw bores
        for( theta=[0,60,120,180,240,300] ) {
          rotate( [0,0,theta+30] )
            translate( [groundplate_outer_screw_position,0,-eps] ) 
              cylinder( d=groundplate_outer_screw_bore, h=hhh+6*eps, $fn=fn, center=false );
        }
        // three (now four!) bottom nut cavities
        for( theta=[60,120,240,300] ) { //true 0,120,240
          rotate( [0,0,theta+30] )
            translate( [groundplate_outer_screw_position,0,-eps] ) 
              cylinder( d=groundplate_outer_nut_diameter,
                        h=groundplate_outer_nut_height, $fn=6, center=false );
        }  
//        // three top nut cavities
//        for( theta=[60,180,300] ) {
//          rotate( [0,0,theta+30] )
//            translate( [groundplate_outer_screw_position,0,hhh-groundplate_outer_nut_height+eps] ) 
//              cylinder( d=groundplate_outer_nut_diameter,
//                        h=groundplate_outer_nut_height+5*eps, $fn=6, center=false );
//        }  
      }
    } // difference
  } // make_groundplate

  

  // baseplate with sensor cutouts and inner hex column,
  // with central screw bore and hex-nut cutout
  if (make_baseplate) 
  difference() {
    // color( [0.99,0.5,0.5] ) 
    color( baseplate_color )
    union() {
      // inner sensor baseplate
      cylinder( d=baseplate_diameter, h=baseplate_thickness, $fn=fn, center=false );

      // central hex column
      rotate( [0,0,30] ) 
        cylinder( r=r_sensor - x_tcst1103/2,
                  h=z_tcst1103+eps, $fn=6, center=false );
      // outer "soldering helper" columns or pillars
      
      if (groundplate_outer_screw_position > 0) {
        // outer screw pillars
        for( theta=[0,60,120,180,240,300] ) {
          rotate( [0,0,theta+30] )
            translate( [groundplate_outer_screw_position,0,eps] ) 
              cylinder( d=groundplate_outer_pillar_diameter, h=z_tcst1103+eps, $fn=fn, center=false );
        }
      } 
      else { // just the "soldering helper" pillars
        for( theta= [0,60,120,180,240,300] ) {
          rotate( [0,0,theta+30] )       
            translate( [baseplate_diameter/2-3, 0, z_tcst1103/2+eps/2] ) 
              cube( size=[3,8,z_tcst1103+eps], center=true );
        }
      }
      
// xxxzzz      
      // dimensions changed from the "thin"/Vishay TCST1103 version,
      // because the ITR9904 is thinner and not so high...
      if (make_baseplate_sensor_guidings == true) {
        ring( d_outer=baseplate_diameter, 
              d_inner=2*(r_sensor+x_itr9904/2-1), $fn=fn, h=z_tcst1103+0.53*eps );
        ring( d_outer=2*(r_sensor-x_itr9904/2+0.5), 
              d_inner=r_sensor-x_tcst1103/2 , $fn=fn, h=z_tcst1103-1.5 );
        echo (r_sensor-x_tcst1103/2 );
      }
    } // union

    // sensor cutouts
    for( theta= [0,60,120,180,240,300] ) {
      rotate( [0,0,theta] )       
        translate( [r_sensor,0,-eps] ) 
          hull() { 
            //vishay_TCST1103( show_wires=false, show_optical_axis=false );
            everlight_ITR9904( show_wires=false );
          }
 
          // minkowski() { vishay_TCST1103(); cube( size=[0.2,0.2,0.2], center=true ); } // compensate for overextruding
    }

    // sensor cutouts, v2 up to top
    for( theta= [0,60,120,180,240,300] ) {
      rotate( [0,0,theta] )       
        translate( [r_sensor,0,
                    groundplate_thickness+groundplate_outer_screw_head_height-z_itr9904] ) 
          hull() { 
            //vishay_TCST1103( show_wires=false, show_optical_axis=false );
            everlight_ITR9904( show_wires=false );
          }
 
          // minkowski() { vishay_TCST1103(); cube( size=[0.2,0.2,0.2], center=true ); } // compensate for overextruding
    }

    // central column bore
    translate( [0,0,-2*eps] )
      cylinder( d=central_bore_diameter, h=z_tcst1103+5*eps, $fn=30, center=false );
    // central nut cavity
    translate( [0,0,-eps] )
      cylinder( d=central_nut_diameter, h=central_nut_height, $fn=6, center=false );

    // horizontal alignment bars on top of central pillar
    translate( [0,eta,z_tcst1103] ) rotate( [0,90,0] )
      cylinder( d=alignment_bar_diameter, h=r_sensor*1.2, $fn=30, center=true );
    translate( [0,0,z_tcst1103] ) rotate( [0,90,90] )
      cylinder( d=alignment_bar_diameter, h=r_sensor*1.2, $fn=30, center=true );

    // horizontal alignment bars on bottom of baseplate
    translate( [0,eta,0] ) rotate( [0,90,0] )
      cylinder( d=alignment_bar_diameter, h=r_sensor*1.2, $fn=30, center=true );
    translate( [0,0,0] ) rotate( [0,90,90] )
      cylinder( d=alignment_bar_diameter, h=r_sensor*1.2, $fn=30, center=true );

    // outer screw pillar bores
    if (groundplate_outer_screw_position > 0) {
      for( theta=[0,60,120,180,240,300] ) {
        // through-bores in outer screw pillars
        rotate( [0,0,theta+30] )
          translate( [groundplate_outer_screw_position,0,-eps] ) 
            cylinder( d=groundplate_outer_screw_bore, h=z_tcst1103+5*eps, $fn=fn, center=false );
      }
      // for( theta=[0,120,240] ) {
      for( theta=[0,60,120,180,240,300] ) {
        // screw head cavities on top of three/all six outer screw pillars
// fnh 2019.01.29 add+1 diameter to remove the (very fragile) outer cylinders for testing
        rotate( [0,0,theta+30] )
          translate( [groundplate_outer_screw_position,
                      0,
                      z_tcst1103-groundplate_outer_screw_head_height-eps] ) 
            cylinder( d=groundplate_outer_screw_head_diameter+1, 
                       h=groundplate_outer_screw_head_height+4*eps, $fn=fn, center=false );
      }
    } 

  } // make_baseplate


  if (make_elastic) {
    h_elastic = elastic_rim_thickness + elastic_rim_z_offset;
    // color( [0.4,0.9,0.4] ) 
  // color( elastic_rim_color )
  translate( [0,0,explode_distance] ) {
    difference() {
      union() {
        // central hex pillar
        translate( [0,0,z_tcst1103-eps] ) rotate( [0,0,30] ) 
          cylinder( r=elastic_pillar_diameter/2, 
                    h=h_elastic,
                    $fn=6, center=false );
        // horizontal alignment bars on bottom of central pillar,
        // compensate length to match diameter of pillar.
        translate( [0,eta,z_tcst1103] ) rotate( [0,90,0] )
          cylinder( d=alignment_bar_diameter, 
                     h=elastic_pillar_diameter*0.85, $fn=30, center=true );
        translate( [0,0,z_tcst1103] ) rotate( [0,90,90] )
          cylinder( d=alignment_bar_diameter, 
                    h=elastic_pillar_diameter*0.85, $fn=30, center=true );
        
        // three connector bars for the spring
        for( theta=[0,120,240] ) {
          intersection() {
            z = z_tcst1103 + h_elastic - elastic_arm_z_thickness/2 -eps;
            rotate( [0,0,theta+30] ) translate( [elastic_pillar_diameter/2,0,z] )
              cube( size=[2.8*elastic_arm_x_thickness, 
                           1*elastic_arm_x_thickness,
                           elastic_arm_z_thickness], center=true );
            translate( [0,0,z-elastic_arm_z_thickness/2] ) 
              cylinder( r=elastic_pillar_diameter/2+1.1*elastic_arm_x_thickness,
                        h=elastic_arm_z_thickness,
                        $fn=fn, center= false );
          } // intersection
        } // for
      } // union

      // central column hole
      translate( [0,0,z_tcst1103-5] )
        cylinder( d=central_bore_diameter, h=h_elastic+5+2*eps, $fn=30, center=false );

      // cavity for central screw head
      translate( [0,0, z_tcst1103 + h_elastic - central_screw_head_height+2*eps] )
        cylinder( d=central_screw_head_diameter,
                  h=central_screw_head_height, $fn=30, center=false );

    } // difference

    // outer elastic rim ring
    // color( [0.4,0.9,0.4] ) 
    if (show_rim) { // hide to better view the "fins"
      difference() {
        translate( [0,0,z_tcst1103+elastic_rim_z_offset] )
          ring( d_outer=elastic_rim_outer_diameter,
                d_inner=elastic_rim_inner_diameter,
                h=elastic_rim_thickness, $fn=fn );
        // six outer rim screw bores
        for( theta=[0,60,120,180,240,300] ) {
          r = (elastic_rim_outer_diameter + elastic_rim_inner_diameter) * 0.25;
          rotate( [0,0,theta+30] ) 
            translate( [r,0,z_tcst1103+elastic_rim_z_offset-eps] )
              cylinder( d=elastic_rim_screw_bore, h=elastic_rim_thickness+2*eps, $fn=fn, center=false );
        }
        // six outer rim hex nut cavities
        for( theta=[0,60,120,180,240,300] ) {
          r = (elastic_rim_outer_diameter + elastic_rim_inner_diameter) * 0.25;
          rotate( [0,0,theta+30] ) 
            translate( [r,0,z_tcst1103+elastic_rim_z_offset-eps] ) rotate( [0,0,30] ) 
              cylinder( d=elastic_rim_nut_diameter, h=elastic_rim_nut_height+2*eps, $fn=6, center=false );
        }

      } // difference
    } // if show_rim

// xxxzzz
if (make_sensor_fins) {
    // sensor fins 
    for( theta=[0,120,240] ) {
      // "z-axis" fins
      // z_tcst1103_beam = 8.4
      h_fin_extra = 0.8; // should not be necessary...
      h_fin = elastic_rim_z_offset+elastic_rim_thickness+(z_tcst1103-z_tcst1103_beam) + h_fin_extra;
      rotate( [0,0,theta] )
        translate( [r_sensor, 0, z_tcst1103+elastic_rim_z_offset+elastic_rim_thickness-h_fin/2] )
          cube( size=[1.5,5,h_fin], center=true );
    }
    for( theta=[60,180,300] ) {
      // "xy-axis" fins
      h_fin = elastic_rim_z_offset+elastic_rim_thickness+(z_tcst1103-z_tcst1103_beam) + 2;
      xy_fin_shift = 0.5; // should be zero...
      xy_fin_width = 4;
      rotate( [0,0,theta] )
        translate( [r_sensor, 
                    -xy_fin_width/2 + xy_fin_shift, 
                    z_tcst1103+elastic_rim_z_offset+elastic_rim_thickness-h_fin/2] )
          cube( size=[1.5,xy_fin_width,h_fin], center=true );
    }
  } // make sensor fins
  
  
  
  
  
  

  zz = z_tcst1103 + elastic_rim_thickness + elastic_rim_z_offset - elastic_arm_z_thickness - eps;

  // outer spiral arm connectors
  for( theta=[0,120,240] ) {
    // v3: theta_offset = -20;
    // v3: xfactor = 1.8;
    theta_offset = 18;
    xfactor = 2.60; // 7.2;
    rotate( [0,0,theta+theta_offset] ) 
      // FNH 2020.01.23: fix x_thickness factor!!!
      translate( [0,
                  elastic_rim_inner_diameter/2+eps,
                  zz+elastic_arm_z_thickness/2] ) 
        cube( size=[2*elastic_arm_x_thickness, xfactor*elastic_arm_x_thickness, elastic_arm_z_thickness], center=true ); // v2: 1.5    
  }
  

  // spiral arms
  // color( [0,0.6,0] ) 
  intersection() {
    translate( [0,0,zz] ) rotate( [0,0,60+3] ) // v2: 60+7
    for( theta=[0,120,240] ) {
      rotate( [0,0,theta] )
        spiral( r=elastic_pillar_diameter/2+1.1*elastic_arm_x_thickness, // v2: 1.1
                thickness=elastic_arm_x_thickness,
                loops=0.48,                  // v3: 0.568 v2: 0.58
                height=elastic_arm_z_thickness );
    }
    // don't make the arms extend into the outer rim ring
    translate( [0,0,zz-eps] ) 
      cylinder( d=elastic_rim_inner_diameter+eps, h=elastic_arm_z_thickness+2*eps, $fn=fn, center=false );
  } // end spiral
 
  

} // translate elastic
  } // make_elastic


  if (make_cover) {

  } // make_cover


  if (make_buttonplate) {
    delta = 30;
    difference() {
      // circular baseplate
      translate( [0,0,-0.3*eps] ) // slight z-offset to keep openscad happy
      ring( d_inner=baseplate_diameter-eps, 
            d_outer=buttonplate_diameter,
            h=buttonplate_thickness, $fn=fn, center=false );

      // a couple of button cutouts
      for( theta= [-delta,0,delta,180-delta,180,180+delta] ) {
        rotate( [0,0,theta] )       
          translate( [r_buttons,0,buttonplate_thickness/2-eps] ) 
            cube( size=[6.3,8,buttonplate_thickness+1], center=true );
      }
    }  

    // the buttons... 
    for( theta= [-delta,0,delta,180-delta,180,180+delta] ) {
      rotate( [0,0,theta] )       
        translate( [r_buttons,0,-eps] ) 
          push_button_simple();
    }
  } // name_buttonplate


  if (make_arduinoplate) {
    // Arduino nano (or Teensy?)
    translate( [0,buttonplate_diameter/2+4*2.54,2] )
      arduino_nano();

    difference() {
      ax = buttonplate_diameter;
      ay = buttonplate_diameter/2 + 7*2.54 + 5;
      az = buttonplate_thickness;
      // arduino carrier plate
      translate( [0,ay/2,az/2-0.7*eps] ) 
        cube( size=[ax,ay,az], center=true );

      // don't overlap the buttonplate 
      translate( [0,0,-0.5] ) 
        cylinder( d=buttonplate_diameter-eps, h=buttonplate_thickness+1, $fn=fn, center=false );
      
      // arduino cutout
      dd = 0.2; // spacing 
      translate( [0,buttonplate_diameter/2+4*2.54,3] )
        cube( size=[17*2.54+dd,7*2.54+dd,8], center=true );

      // TODO: USB-cable cutout!
      echo( "WARNING: missing USB-cable cutout in arduinoplate!" );
    }
  }

  if (make_central_pillar_spacing_plate) {
    hcpsp = h_central_pillar_spacing_plate; // z_tcst1103 + eps;
    difference() {
      union() {
        // central hex column
        rotate( [0,0,30] ) 
          cylinder( r=r_sensor - x_tcst1103/2,
                    h=hcpsp, $fn=6, center=false );
        
        
        // horizontal alignment bars on bottom of baseplate
        translate( [0,eta,0] ) rotate( [0,90,0] )
          cylinder( d=alignment_bar_diameter, h=r_sensor*1.0, $fn=30, center=true );
        translate( [0,0,0] ) rotate( [0,90,90] )
          cylinder( d=alignment_bar_diameter, h=r_sensor*1.0, $fn=30, center=true );
      }

      // central column bore
      translate( [0,0,-2*eps-alignment_bar_diameter] )
        cylinder( d=central_bore_diameter, h=hcpsp+alignment_bar_diameter+5*eps, $fn=30, center=false );

      // horizontal alignment bars on top of central pillar
      translate( [0,eta,hcpsp] ) rotate( [0,90,0] )
        cylinder( d=alignment_bar_diameter, h=r_sensor*1.2, $fn=30, center=true );
      translate( [0,0,hcpsp] ) rotate( [0,90,90] )
        cylinder( d=alignment_bar_diameter, h=r_sensor*1.2, $fn=30, center=true );

    } // difference
  } // spacing_plate




}







// linear_extrude(height = 50, center = true, convexity = 10, twist = -5100, $fn = 50)
// translate([2, 0, 0])
// circle(r = 1);


module spiral( r=5.9, thickness=1.0, loops=1.2, height=1.0 ) {
//r = 5.9;
//thickness = 1.5;
//loops = 1.2;
start_angle = 0;
end_angle = 360 * loops;

function spiral(r, t) = let(r = (r + 2*t / 90)) [r * sin(t), r * cos(t)];

inner = [for(t = [start_angle : end_angle]) spiral(r - thickness, t) ];
outer = [for(t = [end_angle : -1 : start_angle]) spiral(r, t) ];

// This is completely fucked up in OpenSCAD: polygon() cannot
// be used as a function, but it will create an object.
linear_extrude( height=height )
  polygon(concat(inner, outer), convexity=20 ); // specied 2D-path with z (height)=1
}




module sensor_ring_radial() {
  dd = 11.5; // any smaller, and the sensor collide
  for( theta= [0,60,120,180,240,300] ) {
      rotate( [0,0,theta] )       
        translate( [dd,0,-eps] ) vishay_TCST1103();
  }
  translate( [0,0,-3-eps] )
    cylinder( d=36, h=3, center=false );

  color( [1,0,0] ) translate( [0,0,eps] )
    cylinder( d=9, h=15, $fn=50, center=false );

}



module sensor_ring_tangential() {
  dd = 13.9;


  for( theta= [0,60,120,180,240,300] ) {
      rotate( [0,0,theta] )       
        translate( [dd,0,-eps] ) rotate( [0,0,90] ) vishay_TCST1103();
  }
  translate( [0,0,-3-2*eps] )
    cylinder( d=37, h=3, $fn=fn, center=false );

//  translate( [0,0,-5] )
//    ring( d_outer=38, d_inner=37+0.0+eps, h=33, fn=fn, center=false );


  // central hex pillar
  cylinder( d=12,h=11.3,$fn=6, center=false );

  color( [0,0.6,0] ) 
    translate( [0,0,12.3] )
      ring( d_outer=38, d_inner=28, h=5, fn=fn, center=false );

  color( [0,0.6,0] ) 
  translate( [0,0,11.3] )
    cylinder( d=12,h=6,$fn=6, center=false );

  spiral_height = 2.5;
  color( [0,0.6,0] ) 
  translate( [0,0,11.3+6+eps-spiral_height] ) rotate( [0,0,-31] ) 
  for( theta=[0,120,240] ) {
    rotate( [0,0,theta] )
      spiral( height=spiral_height );
  }
}



module sensor_ring_mixed() {
  d1 = 8.8;
  d2 = 10.5;
  for( theta= [0,120,240] ) {
      rotate( [0,0,theta] )       
        translate( [d1,0,-eps] )  vishay_TCST1103();
  }
  for( theta= [60,180,300] ) {
      rotate( [0,0,theta] )       
        translate( [d2,0,-eps] ) rotate( [0,0,90] )  vishay_TCST1103();
  }
  translate( [0,0,-3-eps] )
    cylinder( d=30.6, h=3, center=false );

  color( [1,0,0] ) translate( [0,0,eps] )
    cylinder( d=5, h=15, $fn=50, center=false );
}



module ring( d_outer, d_inner, h, fn=fn, center ) {
  difference() {
    cylinder( d=d_outer, h=h, $fn=fn, center=center );
    translate( [0,0,-eps] ) cylinder( d=d_inner, h=h+2*eps, $fn=fn, center=center );

  }
}



/**
 * simplified convex-hull model of the 2DOF sensor module.
 * To be used for precise "cutouts" on the F/T sensor base
 * plate. Centered on the middle of the first (left) sensor.
 */
module 2DOF_sensor_module_cutout() {
  // sensor base
  bx = 11.9;
  by = 6.3;
  bz = 3.1;
  ez = 10.8;
  translate( [0,0,ez/2] ) cube( size=[bx,by,ez], center=true );
  translate( [0,3*2.54,ez/2] ) cube( size=[bx,by,ez], center=true );
}


/**
 * small plactic holder to help soldering the TCST1103 couplers
 * in precise pose onto the breadboard/sensor PCB.
 */
module 2DOF_sensor_module_soldering_tool() {
  // size of TCST1103 housing
  bx = 11.9;
  by = 6.3;
  bz = 3.1;
  ez = 10.8;

  // correct for overextrusion and irregular FDM surface
  eps = 0.15;

  difference() {
     translate( [0,0,ez/2] ) cube( size=[40,40,ez], center=true);
     translate( [0,-1.5*2.54,ez/2] ) cube( size=[bx+eps,by+eps,ez+eps], center=true );
     translate( [0,+1.5*2.54,ez/2] ) cube( size=[bx+eps,by+eps,ez+eps], center=true );
  }
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
module 2DOF_sensor_module() {
    dx1 =  1.5 * 2.54; // x and y offsets of the optocouplers
    dy1 =  1.5 * 2.54;    
    dy2 =  4.5 * 2.54;    

    translate( [0,0,0] ) breadboard( nx=4, ny=8, z=pcb_thickness, bore=1.0, fn=15, epoxy=false );
    translate( [dx1, dy1, pcb_thickness] ) rotate( [0,0,180] ) vishay_TCST1103();
    translate( [dx1, dy2, pcb_thickness] ) rotate( [0,0,0] ) vishay_TCST1103();
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




// xxxzzz
// translate( [0,0,50] ) spacemouse_nano17_adapter();

module spacemouse_nano17_adapter( ati=true ) {

  elastic_rim_thickness = 7.0;
  elastic_rim_outer_diameter = 42.0; // same as baseplate
  elastic_rim_inner_diameter = 27.0; 
  elastic_rim_screw_bore = 2.6; // M2.5 screw
  elastic_rim_nut_diameter = 5.8; // M2.5 hex nut + eps
  elastic_rim_nut_height = 2.5; // M2.5 hex nut + eps

  nano_adapter_height = 6;
  nano_adapter_inner_height = 2;
  nano_adapter_bolt_head_height = 4;
  nano_adapter_bolt_head_diameter = 5.5;

  theta_nano = 0;

  // align ATi sensor cable and mounting plate with the Teensy USB cable
  // ATi nano17 model is aligned at its tool (upper) face... 
  if (ati) {
    translate( [0,0,nano_adapter_height+14.5] ) 
      rotate( [0,0,theta_nano] ) 
        ati_nano17e();
  }

  translate( [0,0,nano_adapter_height-nano_adapter_inner_height-eps] ) 
    rotate( [0,0,theta_nano] ) 
      ati_nano17e_base_mounting_plate( h=nano_adapter_inner_height, inbus=false, inbus_head_height=1 );

  translate( [0,0,nano_adapter_height-nano_adapter_inner_height-eps] )
    ring( d_outer=elastic_rim_inner_diameter+eps,
          d_inner=17.0-eps, // diameter of sensor
          h=nano_adapter_inner_height, $fn=fn );

  difference() {
    ring( d_outer=elastic_rim_outer_diameter,
          d_inner=elastic_rim_inner_diameter,
          h=nano_adapter_height, $fn=fn );
    // six outer rim screw bores
    for( theta=[0,60,120,180,240,300] ) {
      r = (elastic_rim_outer_diameter + elastic_rim_inner_diameter) * 0.25;
      rotate( [0,0,theta+30] ) 
        translate( [r,0,-eps] )
          cylinder( d=elastic_rim_screw_bore, h=nano_adapter_height+2*eps, $fn=fn, center=false );
    }
    // six outer rim hex bolt head cavities
    for( theta=[0,60,120,180,240,300] ) {
      r = (elastic_rim_outer_diameter + elastic_rim_inner_diameter) * 0.25;
      rotate( [0,0,theta+30] ) 
        translate( [r,0,nano_adapter_height - nano_adapter_bolt_head_height] ) rotate( [0,0,30] ) 
          cylinder( d=nano_adapter_bolt_head_diameter, h=nano_adapter_bolt_head_height+2*eps, $fn=fn, center=false );
    }
  } // difference
}



/**
 * cylindrical plate with six screw bores to match the baseplate
 * and groudplate, and a 10mm diameter cylinder for insertion into
 * our PA-10 robot with drill-chunk adapter.
 * The plate can also just be glued (taped) to the six-axis sensor.
 */
module pa10_drill_chunk_adapter(
  adapter_plate_diameter = 42.0,
  adapter_plate_thickness = 6.0, 
  groundplate_outer_screw_position = 17,
  groundplate_outer_screw_bore = 3.1 + fdm_fudge, // M3 + eps
  groundplate_outer_screw_head_diameter = 6.0 + fdm_fudge, // M3 inbus screw head + eps
  groundplate_outer_screw_head_height = 3.5, // M3 inbus screw head + eps
  groundplate_outer_nut_diameter = 6.4 + fdm_fudge, // M3 hex nut
  groundplate_outer_nut_height = 2.5, // M3 hex nut
  drill_chunk_pin_length = 25.0, 
) 
{
  hhh = adapter_plate_thickness + 2*explode_distance;
  translate( [0,0, -hhh] )
  difference() {
    // main groundplate cylinder
    cylinder( d=adapter_plate_diameter, h=adapter_plate_thickness, $fn=fn, center=false );
       
    // six outer screw bores with nut cavities on top and bottom
    if (groundplate_outer_screw_position > 0) {
      // outer screw bores
      for( theta=[0,60,120,180,240,300] ) {
        rotate( [0,0,theta+30] )
          translate( [groundplate_outer_screw_position,0,-eps] ) 
            cylinder( d=groundplate_outer_screw_bore, h=adapter_plate_thickness+6*eps, $fn=fn, center=false );
      }
    
      for( theta=[0,60,120,180,240,300] ) {
        // screw head cavities on top of three/all six outer screw pillars
        rotate( [0,0,theta+30] )
          translate( [groundplate_outer_screw_position, 0, -2*eps] )
            cylinder( d=groundplate_outer_screw_head_diameter+1, 
                       h=groundplate_outer_screw_head_height+4*eps, $fn=6, center=false );
      }
    } 
    
    // +x axis marker
    translate( [adapter_plate_diameter/2, 0, -eps] )
      cylinder( d=1, h=adapter_plate_thickness+4*eps, center=false, $fn=10 );
  } // difference

  // cylinder to fit into the drill-chunk 
  translate( [0,0, -hhh - drill_chunk_pin_length +eps] )
    cylinder( d=10.0, h=drill_chunk_pin_length, center=false, $fn=50 ); 
    
}





module coin_calibration_rig(
  d_outer = 42, d_inner = 27,
  h       = 7,
  l_arms  = 100.0,  
  d_arms  =   7.0,
  h_arms  =   8.0,
)
{
  M25BORE = 2.6 + fdm_fudge; // M2.5 screw
  
  D2CENT  = 18.75;
  H2CENT  =  1.67;
  M5CENT  =  3.06;
  
  D5CENT  = 21.25; // similar to US 5 cents and CAN 5 cents...
  H5CENT  =  1.67;
  M5CENT  =  3.92; // grams
  
  hcyl = 25 * H5CENT;

  difference() {
    union() {
      ring( d_outer=d_outer, d_inner=d_inner, h=h, $fn=200, center=false );
      for( theta=[0,90,180,270] ) {
        rotate( [0,0,theta+30] ) 
          translate( [l_arms/2, 0, h_arms/2] ) 
            cube( size=[l_arms, d_arms, h_arms], center=true );
        rotate( [0,0,theta+30] ) 
          translate( [l_arms, 0, 0] ) 
            cylinder( d=24.75, h=hcyl, center=false, $fn=100 );
      }
    }
    // six outer rim screw bores
    for( theta=[0,60,120,180,240,300] ) {
      r = (d_outer + d_inner) * 0.25;
      rotate( [0,0,theta+30] ) 
         translate( [r,0,-eps] )
           cylinder( d=M25BORE, h=max(h,h_arms)+2*eps, $fn=fn, center=false );
    }
    // cutouts of the coin cylinders
    for( theta=[0,90,180,270] ) {
      rotate( [0,0,theta+30] ) 
        translate( [l_arms,0,1] ) cylinder( d=21.75, h=hcyl, center=false, $fn=100 );
      rotate( [0,0,theta+30] ) 
        translate( [l_arms,0,-eps] ) cylinder( d=5, h=3, center=false, $fn=100 );

      rotate( [0,0,theta+30] ) 
        translate( [l_arms,0,hcyl-15+eps] ) cube( size=[8,25,30], center=true );
      rotate( [0,0,theta+30] ) 
        translate( [l_arms,25/2,0] ) cube( size=[3,4,10], center=true );
      rotate( [0,0,theta+30] ) 
        translate( [l_arms,-25/2,0] ) cube( size=[3,4,10], center=true );
    }
  }
}


// translate( [0,70,70] ) coin_calibration_stops();


module coin_calibration_stops( d_outer=21.8, h_outer=10, h_inner=40, t_plate=1.3, t_wall=1 ) {
  // bottom plate
  difference() {
    cylinder( d=13, h=h_inner, center=false, $fn=100 );
    translate( [0,0,h_inner/2+t_plate/2] )
      cube( size=[d_outer+2+eps, 4, h_inner-t_plate+eps], center=true );
    translate( [0,6.5, h_inner/2+t_plate/2] )
      cube( size=[4, 2, h_inner-t_plate+eps], center=true );
    translate( [0,-6.5, h_inner/2+t_plate/2] )
      cube( size=[4, 2, h_inner-t_plate+eps], center=true );
    
  }
  difference() {
    cylinder( d1=d_outer-0.5, d2=d_outer+0.5, h=h_outer, center=false, $fn=100 );
    translate( [0,0,t_plate] )
      cylinder( d=d_outer-2*t_wall, h=h_outer, center=false, $fn=100 );
    
    translate( [0,0,h_outer/2+t_plate/2] )
      cube( size=[d_outer+2+eps, 5, h_outer-t_plate+eps], center=true );
    
    
  }
}


module coin_calibration_piston( d_plate=21.0, t_plate=1, d_lever=2, h_lever=40 )
{
  cylinder( d=d_plate, h=t_plate, center=false, $fn=100 );
  cylinder( d=d_lever, h=h_lever, center=false, $fn=100 );
}

         



/**
 * cylinder that fits a nut of given metric size,
 * e.g. m=4 means M4.
 * https://www.boltdepot.com/fastener-information/nuts-washers/Metric-Nut-Dimensions.aspx
 * diameter is across the flats, also the size of wrench to use.
 * M2: diameter 4.0, height 1.6
 * M2.5: 5.0 2.0
 * M3:   5.5 2.4
 * M4:   7.0 3.2
 * M5:   8.0 4.0
 * M6:   10.0 5.0
 * ...
 */
module metric_nut_mount( outer_diameter=20, thickness=3.5, m=7.0, fn=30, eps=0.01 ) 
{
  color( [1,1,0] )
  difference() {
    cylinder( d=outer_diameter, h=thickness+eps, center=true, $fn=fn );
    cylinder( d=m, h=thickness+2*eps, center=true, $fn=6 );
  }
}




// include <breadboard.scad>
include <optokoppler.scad>
include <electronics.scad>
include <ati-nano17e.scad>


