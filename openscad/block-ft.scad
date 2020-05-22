/** "block-ft" version w3
 * 
 * Parametric 3D-printed 6-axis F/T sensor based on eight
 * optical IR proximity/reflex sensors (Everlight ITR8307).
 *
 * The design uses a cross-shaped "swastika"-style 
 * elastic member, with black/white contrast sensing 
 * at fixed z-distance on the inner arms, and variable 
 * z-distance proximity sensing on the outer arms.
 * 
 * The sensor is mounted to the robot using (up to) six
 * screws on the central pillar, and four smaller screws
 * on the tool side.
 * 
 * See the corresponding Eagle schematics and layouts
 * for Everlight ITR 8307 sensor layout and readout
 * using an Arduino Nano microcontroller.
 *
 * 2020.03.06 - sunrise M3207 adapter plate
 * 2020.03.04 - w3 solder the sensor plate, testing
 * 2020.03.01 - make cover 
 * 2020.03.01 - actually print the thing...
 *
 * 2017.03.02 - w2 quadruple outer arm, triangle stiffeners
 * 2017.01.23 - w1 nested two-point-suspended levers
 * 
 * (C) 2017, 2020 fnh, hendrich@informatik.uni-hamburg.de
 */


include <optokoppler.scad>


// NOTE: dimensions in millimeters! Use an outer scale( 0.001 ) 
// if meters are required (e.g., conversion to ROS URDF).

eps                       = 0.01;     // a small constant for overlap etc.
fn_global                 = 20;       // number of faces on cylinders/spheres
extrusion_width           = 0.35;
explode_distance          = 22.0;      // you want 0 for printing, 30 for preview
use_colors                = true;


// select which main parts to create
// 
make_base_plate           = true;    // "robot" side, you want this
make_elastic_member       = true;    // you definitely need this
make_cover_plate          = false;   // "tool" side
make_cover_ring           = true;    // "tool" side with sensor fins
make_fake_pcb             = true;    // shows sensor layout, you want "false" for 3D-printing
make_upper_pcb            = false;

make_pattern_plate        = true;
make_spacer_plate         = true;
make_base_holder          = true;
make_sunrise_m3207_plate  = true;


// select part options
// 
base_plate_cable_channel_width = 4;   // 0.. 
base_plate_groove_diameter     = 1;   // groove/notch for form closure
third_arm_stiffener_thickness  = 2;   // triangular stiffeners 

make_lever_plate          = false;
make_thin_sensor_arms = false; // 
has_amplifier_arm = true;


// for (i = [start:step:end]) { … }





base_diameter       = 2*26+4;  // outer diameter of sensor
base_thickness      = 3;   // base plate thickness
base_support_pillar_thickness = 3; // core pillar stubs on base plate
base_wall_width     = 1;   // outer base wall for (light) protection,
base_wall_height    = 1; // 14  // outer base wall for (light) protection,

base_screw_diameter = 6.2; // M3 flachkopf
base_screw_cavity   = 2.0; // M3 flachkopf: 1.8
 
pcb_diameter        = 55;
pcb_thickness       = 1;   // pcb board thickness

core_pillar_height    = 12;
core_pillar_diameter  = 8.0;
core_pillar_dx        = 8.0;
core_pillar_dy        = 8.0;
core_pillar_screw_bore= 3.2; // M3 3.2=3.0+eps M4 4.2 + eps
core_nut_diameter     = 6.3; // outer diameter of mounting nut (e.g. M4 = 7.0 M3 = 6.08+eps)
core_nut_height       = 3.5; // height of mounting nut cavity (e.g. M4 = 3.2 M3 = 2.5)

inner_arm_thickness   = 0.7; // 1.0;
inner_arm_length      = 11;
inner_arm_height      = 10.0;
inner_arm_separation  = 13.0;  // distance between inner "x-deflection" arms (red)

middle_arm_thickness  = 0.7; // inner_arm_thickness;
middle_arm_length     = 8;
middle_arm_height     = 10.0;

third_arm_thickness   = 2;     // minimum thickness of third arm
third_arm_height      = 10;    // height
third_arm_extra_thickness = 1; // extra thickness in the midtruedle of third arm

outer_arm_sep         = 1.5;
outer_arm_thickness   = 0.6;
outer_arm_length      = 10;  // was 12 in version w1
outer_arm_width       = 4;
outer_arm_yyy         = 16;  // width of connector from third arm to outer arm

outer_pillar_height   = 12;
outer_pillar_diameter = 8.0;
outer_pillar_screw_bore = 3.2; // M3 + eps
outer_nut_diameter    = 6.3;   // M3 nut + eps
outer_nut_height      = 4.0;   // M3 nut (full height)

cover_plate_thickness = 4.0;
cover_screw_diameter  = 6.5; // M3 flachkopf DIN 85: head diameter 6.0
cover_screw_cavity    = 2.0; // M3 flachkopf DIN 85: head thickness 1.8

cover_ring_thickness  = 8;
cover_ring_inner_diameter = 40;

// instantiate one sensor, with params as given above
// 

translate( [0,0,0] ) six_axis_ft_sensor_w1();



module six_axis_ft_sensor_w1() 
{
  z_base    = 0;
  z_elastic = base_thickness + base_support_pillar_thickness + 1*explode_distance;
  z_cover   = z_elastic + outer_pillar_height + eps + 1.6*explode_distance;
  z_pattern = z_elastic + outer_pillar_height + cover_ring_thickness -2 + 2.5*explode_distance;
  z_pcb     = z_elastic + core_pillar_height + 1 + 0.5*explode_distance; // FIXME!!
  
  if (make_base_plate)     { translate( [0,0,z_base] )    base_plate(); }
  if (make_fake_pcb)       { translate( [0,0,z_pcb]  )    fake_pcb(); }
  if (make_lever_plate)    { }
  if (make_elastic_member) { translate( [0,0,z_elastic] ) elastic_member(); }
  if (make_cover_plate)    { translate( [0,0,z_cover] )   cover_plate(); }
  if (make_cover_ring)     { translate( [0,0,z_cover] )   cover_ring(); }
  if (make_upper_pcb)      { translate( [0,0,z_cover] )   upper_pcb(); }
  if (make_pattern_plate)  { translate( [0,0,z_pattern] ) pattern_plate(); }
  
  if (make_base_holder)    { translate( [0,-50, 0] ) translate( [0,0,-1] ) rotate( [0,180,0] ) base_holder( h=30, hscrew=5 ); }
  if (make_spacer_plate)   { translate( [0,50,0] ) spacer_plate( height=1 ); 
                             translate( [0,80,0] ) spacer_plate( height=1.5 );
                             translate( [0,110,0] ) spacer_plate( height=2.0 );
  }
  if (make_sunrise_m3207_plate) { translate( [100,0,-0] ) rotate( [0,0,0] ) sunrise_M3207_adapter_plate(); }
  if (make_sunrise_m3207_plate) { translate( [0,0,-10] ) rotate( [0,0,0] ) sunrise_M3207_base_holder(); }

} // end six_axis_ft_sensor


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


module screw_M4( length=10, head_length=3, head_diameter=7,
                 washer_thickness=0, washer_diameter=10 ) {
  r = 2.1; eps=0.1;
  union() {
    cylinder( d=head_diameter, h=head_length+eps, center=false, $fn=20 ); 
    translate( [0,0,head_length] ) cylinder( r=r, h=length, center=false, $fn=20 );
    translate( [0,0,head_length] ) cylinder( d=washer_diameter, h=washer_thickness, center=false, $fn=20 );
  }
}


/**
 * the rectangular base plate with outer wall for protection
 * and inner core "column" for force-closure with the elastic
 * member.
 *
 * TODO: cable "outlet"
 */
module base_plate()
{
  union() {
    difference() {
      gray04() 
      union() {
        // main rectangular base plate
        //
        translate( [0,0, base_thickness/2] )
          cube( size=[base_diameter, base_diameter, base_thickness], center=true );      
     
        // stub pillars on base to compensate for pcb-thickness, and also
        // to allow elastic-member to be printed without support materials
        //
        x1 = 2*core_pillar_dx + 2*core_pillar_diameter/2;
        y1 = 2*core_pillar_dy + 2*core_pillar_diameter/2;
        translate( [0,0, base_thickness+base_support_pillar_thickness/2] )
          cube( size=[x1, y1, base_support_pillar_thickness], center=true );

        // diagonal groove for form-closure between base and elastic member
        //
        if (base_plate_groove_diameter > 0) {
          for( ix = [-1,1] ) {
            for( iy= [-1,1] ) {
              translate( [ix*core_pillar_dx, iy*core_pillar_dy, base_thickness+base_support_pillar_thickness] )
                rotate( [90,0,-1*ix*iy*45] )
                  cylinder( d=base_plate_groove_diameter, h=1.2*core_pillar_diameter, center=true, $fn=fn_global ); 
            }
          }
        }
      } // end main rectangular base plate
        
      // through screw holes for the core pillars
      // 
      z2 = base_thickness/2 + base_support_pillar_thickness/2;
      h2 = base_thickness + base_support_pillar_thickness + base_plate_groove_diameter + 2*eps;
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, z2] ) 
            cylinder( d=core_pillar_screw_bore, h=h2, center=true, $fn=fn_global );
        }
      }
      
      // screw head cavity holes on inner screw bores
      //
      for( ix=[-1,+1] ) {
       translate( [ix*core_pillar_dx, 0, base_screw_cavity/2] ) 
         cylinder( d=base_screw_diameter, h=base_screw_cavity+2*eps, center=true, $fn=fn_global ); 
      }

      // inner bore  
      // 
      xx = 2*(core_pillar_dx - core_pillar_diameter/2);
      yy = 2*(core_pillar_dy - core_pillar_diameter/2);
      translate( [0,0,base_thickness/2+base_support_pillar_thickness/2] )
        cube( size=[xx,yy,base_thickness+base_support_pillar_thickness+2*eps], center=true );
      
      // optional cable channel to inner bore
      // 
      if (base_plate_cable_channel_width > 0) {
        yyy = 2*core_pillar_dy + core_pillar_diameter + 2*eps;
        translate( [0,0,base_thickness + base_support_pillar_thickness/2] )
          cube( size=[base_plate_cable_channel_width, yyy, base_support_pillar_thickness+eps], center=true );
      }
    } // base plate with screw bores and inner bore
      
    // thin outer wall for (ambient light) protection    translate( [0, y2, z2] ) 
    // 
    difference() {
      z = base_thickness + base_wall_height/2;
      translate( [0,0,z] )
        gray04()
          cube( size=[base_diameter, base_diameter, base_wall_height], center=true );
      translate( [0,0,z] )     
        cube( size=[base_diameter-2*base_wall_width, base_diameter-2*base_wall_width, base_wall_height+2*eps], center=true );
    }

  }
} // end base_plate



module elastic_member() 
{
  union() {
    // inner support structure: four inner pillars plus connectors,
    // or main block, minus inner "bore".
    // Also includes the through-holes for the screws and the nut cavities.
    //

    difference() {
      // single main block with central bore
      //
      gray05() // color( [0.9,0.6,0.9] ) 
      difference() {
        x1 = 2*core_pillar_dx + 2*core_pillar_diameter/2;
        y1 = 2*core_pillar_dy + 2*core_pillar_diameter/2;
        x2 = 2*core_pillar_dx - 2*core_pillar_diameter/2;
        y2 = 2*core_pillar_dy - 2*core_pillar_diameter/2;
        translate( [0,0,core_pillar_height/2] )
          cube( size=[x1,y1,core_pillar_height], center=true );
        
        translate( [0,0,core_pillar_height/2] )
          gray06()
            cube( size=[x2,y2,core_pillar_height+2*eps], center=true );
      }
  
      // four through holes for the screws
      // 
      hx = core_pillar_height + 2*eps;
      gray05()
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, core_pillar_height/2] ) 
            cylinder( d=core_pillar_screw_bore, h=hx, center=true, $fn=fn_global );
        }
      }

      hz = core_pillar_height - core_nut_height/2;
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          // nut cavity for the mounting screw nut
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, hz] ) 
           gray05()
             cylinder( d=core_nut_diameter, h=core_nut_height+2*eps, center=true, $fn=6 );
        }
      }

      // inner bore  
      // 
      xx = 2*(core_pillar_dx - core_pillar_diameter/2);
      yy = 2*(core_pillar_dy - core_pillar_diameter/2);
      translate( [0,0,core_pillar_height/2] )
        cube( size=[xx,yy,core_pillar_height+2*eps], center=true );
      
      // diagonal groove for form-closure between base and elastic member
      //
      if (base_plate_groove_diameter > 0) {
        for( ix = [-1,1] ) {
          for( iy= [-1,1] ) {
            translate( [ix*core_pillar_dx, iy*core_pillar_dy, 0] )
              rotate( [90,0,-1*ix*iy*45] )
                cylinder( d=base_plate_groove_diameter, h=1.4*core_pillar_diameter, center=true, $fn=fn_global ); 
          }
        }
      }
    } // core part
    
    // inner arms for x-movements
    //
    
    // NOTE: x1 decides the distance between the inner elastic bars!!!
    x1 = inner_arm_separation/2; // core_pillar_dx - core_pillar_diameter/4; 
    y1 = core_pillar_dy + core_pillar_diameter/2 + inner_arm_length/2;
    z1 = inner_arm_height/2;
    for( ix = [-1,1] ) {
      for( iy= [-1,1] ) {
        translate( [ix*x1, iy*y1, z1] ) 
          color( [1,0,0] ) 
            cube( size=[inner_arm_thickness, inner_arm_length, inner_arm_height], center=true );
      }
    }
    
    // full-length middle arms for y-movements
    //
    x2 = x1 + middle_arm_length;
    y2 = core_pillar_dy + core_pillar_diameter/2 + inner_arm_length + middle_arm_thickness/2 - eps;
    z2 = middle_arm_height/2;
    translate( [0, y2, z2] ) 
      color( [0,1,0] ) 
        cube( size=[2*x2, middle_arm_thickness, middle_arm_height], center=true );
    translate( [0, -y2, z2] ) 
      color( [0,1,0] ) 
        cube( size=[2*x2, middle_arm_thickness, middle_arm_height], center=true );
    
    // full-length middle-arm connectors
    //
    x3 = x2 + third_arm_thickness/2;
    y3 = core_pillar_dy + core_pillar_diameter/2 + inner_arm_length + middle_arm_thickness;
    z3 = third_arm_height/2;

    gray06()
    for( ix = [-1,1] ) {
      translate( [ix*x3, 0, z3] ) 
        cube( size=[third_arm_thickness, 2*y3, third_arm_height], center=true );
      translate( [ix*(x3-third_arm_thickness/2), 0, z3] )
        cube( size=[third_arm_thickness, 2*y3-2*third_arm_thickness, third_arm_height], center=true );
    }
    

    // outer arms (without outer pillars)
    //
    x4  = x3 + third_arm_thickness/2 + outer_arm_sep/2;
    x4b = x4 + outer_arm_sep/2 + outer_arm_width/2;
    y4  = core_pillar_dy + core_pillar_diameter/2 + inner_arm_length/2;
    dy4 = outer_arm_length/2 + outer_arm_yyy/2;
    z4  = inner_arm_height/2;
    for( ix = [-1,1] ) {
      for( iy= [-1,1] ) {
        // third arm to outer arm connector 
        //
        translate( [ix*x4, 0, third_arm_height/2] )
          gray06()
            cube( size=[outer_arm_sep, outer_arm_yyy, third_arm_height], center=true );
        translate( [ix*x4b, 0, third_arm_height/2] )
          gray06()
            cube( size=[outer_arm_width, outer_arm_yyy, third_arm_height], center=true );
        
        // actual outer arm (z-deflection): w3: double outer arms near top and near bottom
        // 
        translate( [ix*x4b, iy*dy4, third_arm_height-3*outer_arm_thickness/2] ) // FIXME!
          color( [0,0,1] )
            cube( size=[outer_arm_width, outer_arm_length, outer_arm_thickness], center=true );

        // bottom arm directly on printbed
        translate( [ix*x4b, iy*dy4, outer_arm_thickness/2] ) 
          color( [0,0,1] )
            cube( size=[outer_arm_width, outer_arm_length, outer_arm_thickness], center=true );
      }
    }
    
    if (third_arm_stiffener_thickness > 0) {
      dx  = outer_arm_width + outer_arm_sep;   // stiffener x size
      dy  = outer_arm_length-3;                // stiffener y size
      // zs1 = third_arm_stiffener_thickness/2;
      // zs2 = third_arm_height - third_arm_stiffener_thickness/2;
      zs0 = third_arm_height/2;
      xs  = x4 + outer_arm_width/2;
      ys  = outer_arm_yyy/2 + dy/2;
      
      /*
      // version w2: double cubes as stiffeners near bottom and near top of outer arm
      //
      color( [0.7, 0.7, 0.5] )
      for( ix = [-1,1] ) {
        for( iy= [-1,1] ) {
          // third arm to outer arm connector 
          //
          translate( [ ix*xs, iy*ys, zs1] )
            cube( size=[dx,dy,third_arm_stiffener_thickness], center=true );
          translate( [ ix*xs, iy*ys, zs2] )
            cube( size=[dx,dy,third_arm_stiffener_thickness], center=true );
        }
      }
      */
      
      // version w3: single stiffener at zs0 instead of two stiffeners at z=zs1,zs2
      // also, triangular stiffener instead of square one for easier overhanging printing
      //
      ph = dy;
      pw = dx;
      pl = third_arm_stiffener_thickness*2;
          
      translate( [ -xs, -ys, zs0] ) rotate( [+90, +90, 0 ] ) translate( [-pl/2, -pw/2, -ph/2] )            prism( pl, pw, ph );
      translate( [ -xs, +ys, zs0] ) rotate( [-90, -90, 0 ] ) translate( [-pl/2, -pw/2, -ph/2] )            prism( pl, pw, ph );

      translate( [ +xs, -ys, zs0] ) rotate( [+90, -90, 0 ] ) translate( [-pl/2, -pw/2, -ph/2] )            prism( pl, pw, ph );
      translate( [ +xs, +ys, zs0] ) rotate( [-90, +90, 0 ] ) translate( [-pl/2, -pw/2, -ph/2] )            prism( pl, pw, ph );
    }
    

    // outer pillars with screw hole / nut cavity 
    // 
    x5 = x4 + outer_arm_sep/2 + outer_pillar_diameter/2;
    y5 = outer_arm_yyy/2 + outer_arm_length + outer_pillar_diameter/2;
    
    echo( " " );
    echo( "OUTER PILLARS ARE AT (x,y)= ", x5, ", ", y5 );
    echo( " " );
    
    for( ix = [-1,1] ) {
      for( iy= [-1,1] ) {
        translate( [ix*x5, iy*y5, 0] )
          gray08()
            outer_pillar();
      }
    }
    
  }
} // elastic member



module outer_pillar() {
  color( [0.7, 0.3, 0.8]) 
  difference() {
    // the main pillar
    translate( [0,0, outer_pillar_height/2] )
      cube( size=[outer_pillar_diameter, outer_pillar_diameter, outer_pillar_height], center=true );
    
    // through-hole for the mounting screws
    translate( [0,0, outer_pillar_height/2] )
      cylinder( d=outer_pillar_screw_bore, h=outer_pillar_height+2*eps, center=true, $fn=fn_global );
    
    // nut cavity for the mounting screw nut
    translate( [0,0,outer_nut_height/2] )
      cylinder( d=outer_nut_diameter, h=outer_nut_height+2*eps, center=true, $fn=6 );
  } 
}


/* **************************************************************** */
/* **************************************************************** */
/* SPACER PLATE and BASE HOLDER                                     */
/* **************************************************************** */
/* **************************************************************** */


module spacer_plate( height=1 ) 
{
  core_pillar_height = height;
      #difference() {
      // single main block with central bore
      //
      gray05() // color( [0.9,0.6,0.9] ) 
      difference() {
        x1 = 2*core_pillar_dx + 2*core_pillar_diameter/2;
        y1 = 2*core_pillar_dy + 2*core_pillar_diameter/2;
        x2 = 2*core_pillar_dx - 2*core_pillar_diameter/2;
        y2 = 2*core_pillar_dy - 2*core_pillar_diameter/2;
        translate( [0,0,core_pillar_height/2] )
          cube( size=[x1,y1,core_pillar_height], center=true );
        
        translate( [0,0,core_pillar_height/2] )
          gray06()
            cube( size=[x2,y2,core_pillar_height+2*eps], center=true );
      }
  
      // four through holes for the screws
      // 
      hx = core_pillar_height + 2*eps;
      gray05()
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, core_pillar_height/2] ) 
            cylinder( d=core_pillar_screw_bore, h=hx, center=true, $fn=fn_global );
        }
      }

      hz = core_pillar_height - core_nut_height/2;
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          // nut cavity for the mounting screw nut
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, hz] ) 
           gray05()
             cylinder( d=core_nut_diameter, h=core_nut_height+2*eps, center=true, $fn=6 );
        }
      }

      // inner bore  
      // 
      xx = 2*(core_pillar_dx - core_pillar_diameter/2);
      yy = 2*(core_pillar_dy - core_pillar_diameter/2);
      translate( [0,0,core_pillar_height/2] )
        cube( size=[xx,yy,core_pillar_height+2*eps], center=true );
      
    } // core part
}


/**
 * central block with six screw holes and inner rectangular hole.
 * Nut bores all the length except the last few "hscrew" millimeters.
 */
module base_holder( h=15, hscrew = 5 ) {
      core_pillar_height = h;
      difference() {
      // single main block with central bore
      //
      gray05() // color( [0.9,0.6,0.9] ) 
      difference() {
        x1 = 2*core_pillar_dx + 2*core_pillar_diameter/2;
        y1 = 2*core_pillar_dy + 2*core_pillar_diameter/2;
        x2 = 2*core_pillar_dx - 2*core_pillar_diameter/2;
        y2 = 2*core_pillar_dy - 2*core_pillar_diameter/2;
        translate( [0,0,core_pillar_height/2] )
          cube( size=[x1,y1,core_pillar_height], center=true );
        
        translate( [0,0,core_pillar_height/2] )
          gray06()
            cube( size=[x2,y2,core_pillar_height+2*eps], center=true );
      }
  
      // four through holes for the screws
      // 
      hx = core_pillar_height + 2*eps;
      gray05()
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, core_pillar_height/2] ) 
            cylinder( d=core_pillar_screw_bore, h=hx, center=true, $fn=fn_global );
        }
      }

      core_nut_height  = core_pillar_height - hscrew;
      hz = core_pillar_height - core_nut_height/2;
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          // nut cavity for the mounting screw nut
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, hz] ) 
           gray05()
             cylinder( d=core_nut_diameter, h=core_nut_height+2*eps, center=true, $fn=6 );
        }
      }

      // inner bore  
      // 
      xx = 2*(core_pillar_dx - core_pillar_diameter/2);
      yy = 2*(core_pillar_dy - core_pillar_diameter/2);
      translate( [0,0,core_pillar_height/2] )
        cube( size=[xx,yy,core_pillar_height+2*eps], center=true );
      
      // diagonal groove for form-closure between base and elastic member
      //
      if (base_plate_groove_diameter > 0) {
        for( ix = [-1,1] ) {
          for( iy= [-1,1] ) {
            translate( [ix*core_pillar_dx, iy*core_pillar_dy, 0] )
              rotate( [90,0,-1*ix*iy*45] )
                cylinder( d=base_plate_groove_diameter, h=1.4*core_pillar_diameter, center=true, $fn=fn_global ); 
          }
        }
      }
    } // core part
  
}





/* **************************************************************** */
/* **************************************************************** */
/* COVER RING                                                       */
/* **************************************************************** */
/* **************************************************************** */



/**
 * the rectangular cover "ring" with four screw holes.
 */
module cover_ring() {
  difference() {
    z = cover_ring_thickness;
    s1 = base_diameter;
    s2 = cover_ring_inner_diameter;
    
    gray08()
    translate( [0, 0, z/2] )
      cube( size=[s1, s1, z], center=true );

    translate( [0, 0, z/2] )
      cube( size=[s2, s2, z+2*eps], center=true );

    // four screw bores with cavity for a (flat head, DIN 85) screw
    //
    x5 = inner_arm_separation/2 + middle_arm_length + third_arm_thickness + outer_arm_sep + outer_pillar_diameter/2;
    y5 = outer_arm_yyy/2 + outer_arm_length + outer_pillar_diameter/2;
    gray08()
    for( ix = [-1,1] ) {
      for( iy= [-1,1] ) {
        translate( [ix*x5, iy*y5, z/2] )
          cylinder( d=outer_pillar_screw_bore, h=z+2*eps+30, center=true, $fn=fn_global );
        translate( [ix*x5, iy*y5, z-cover_screw_cavity/2] )
          cylinder( d=cover_screw_diameter, h=cover_screw_cavity+2*eps, center=true, $fn=fn_global );
      }
    }
    
    // pattern-plate alignment notches
    nz = 2.0; sz = (s1-s2)/2+1;
    for( theta=[0,90,180,270] ) {
      rotate( [0,0,theta] ) 
        translate( [(s1+s2)/4,0,z-nz/2] )
          cube( size=[sz,sz,nz+2*eps], center=true );
    }
    
    // inner bore  
    // 
    xx = 2*(core_pillar_dx - core_pillar_diameter/2);
    yy = 2*(core_pillar_dy - core_pillar_diameter/2);
    gray08()
    translate( [0,0,cover_plate_thickness/2] )
      cube( size=[xx,yy,cover_plate_thickness+2*eps], center=true );
  }
}


/* **************************************************************** */
/* **************************************************************** */
/* PATTERN PLATE                                                    */
/* **************************************************************** */
/* **************************************************************** */


module pattern_plate( pattern_plate_thickness=1.0 ) {
  z  = pattern_plate_thickness;
  s1 = base_diameter;
  s2 = cover_ring_inner_diameter-2;
    
  translate( [0, 0, z/2] )
    cube( size=[s2, s2, z], center=true );

  // pattern-plate alignment notches
  sz = (s1-s2)/2-1;
  for( theta=[0,90,180,270] ) {
    rotate( [0,0,theta] ) 
      translate( [(s1+s2)/4-1,0,z/2] )
        cube( size=[sz+1,sz,z-eps], center=true );
  }
}


/* **************************************************************** */
/* **************************************************************** */
/* UPPER PCB  (breadboard)                                          */
/* **************************************************************** */
/* **************************************************************** */



module upper_pcb() {
  difference() {
    union() {
      breadboard( nx=14, ny=14, z=1.0, fn=11, delta=2.54, single=true, copper=true, center=true );
      dz = 1.1;
      translate( [ 1.5*2.54, 6*2.54,dz] ) rotate( [0,0,180] ) everlight_ITR8307( wire_length=1 );
      translate( [-1.5*2.54, 6*2.54,dz] ) rotate( [0,0,180] ) everlight_ITR8307( wire_length=1 );
      translate( [ 1.5*2.54,-6*2.54,dz] ) rotate( [0,0,0] )   everlight_ITR8307( wire_length=1 );
      translate( [-1.5*2.54,-6*2.54,dz] ) rotate( [0,0,0] )   everlight_ITR8307( wire_length=1 );

      translate( [ 6*2.54,-1.5*2.54,dz] ) rotate( [0,0,90] )  everlight_ITR8307( wire_length=1 );
      translate( [-6*2.54,-1.5*2.54,dz] ) rotate( [0,0,270] ) everlight_ITR8307( wire_length=1 );
      translate( [ 6*2.54, 1.5*2.54,dz] ) rotate( [0,0,90] )  everlight_ITR8307( wire_length=1 );
      translate( [-6*2.54, 1.5*2.54,dz] ) rotate( [0,0,270] ) everlight_ITR8307( wire_length=1 );
   
      // cube( [16,16,2], center=true ); // alignment check
    }
 
    // through screw holes for the core pillars
    // 
    z2 = base_thickness/2 + base_support_pillar_thickness/2;
    h2 = base_thickness + base_support_pillar_thickness + base_plate_groove_diameter + 2*eps;
    for( ix = [-1,1] ) {
      for( iy= [-1,1] ) {
        translate( [ix*core_pillar_dx, iy*core_pillar_dy, z2] ) 
          cylinder( d=core_nut_diameter, h=h2, center=true, $fn=fn_global );
      }
      translate( [ix*core_pillar_dx, 0*core_pillar_dy, z2] ) 
        cylinder( d=core_pillar_screw_bore, h=h2, center=true, $fn=fn_global );
    }
    
    // central bore for cable
    cylinder( d=core_nut_diameter, h=h2, center=true, $fn=fn_global );
  }
}






/* **************************************************************** */
/* **************************************************************** */
/* COVER PLATE                                                      */
/* **************************************************************** */
/* **************************************************************** */



/**
 * the rectangular cover plate with four screw holes.
 * Not needed if your tool already includes matching 
 * mouting holes.
 */
module cover_plate() {
  difference() {
    z = cover_plate_thickness;
    gray08()
    translate( [0, 0, z/2] )
      cube( size=[base_diameter, base_diameter, z], center=true );

    // four screw bores with cavity for a (flat head, DIN 85) screw
    //
    x5 = inner_arm_separation/2 + middle_arm_length + third_arm_thickness + outer_arm_sep + outer_pillar_diameter/2;
    y5 = outer_arm_yyy/2 + outer_arm_length + outer_pillar_diameter/2;
    gray08()
    for( ix = [-1,1] ) {
      for( iy= [-1,1] ) {
        translate( [ix*x5, iy*y5, z/2] )
          cylinder( d=outer_pillar_screw_bore, h=z+2*eps+30, center=true, $fn=fn_global );
        translate( [ix*x5, iy*y5, z-cover_screw_cavity/2] )
          cylinder( d=cover_screw_diameter, h=cover_screw_cavity+2*eps, center=true, $fn=fn_global );
      }
    }
    
    // inner bore  
    // 
    xx = 2*(core_pillar_dx - core_pillar_diameter/2);
    yy = 2*(core_pillar_dy - core_pillar_diameter/2);
    gray08()
    translate( [0,0,cover_plate_thickness/2] )
      cube( size=[xx,yy,cover_plate_thickness+2*eps], center=true );
  }
}

/*
  
      // the extensions of the outer pillars from the elastic member
      //
      for( i=[0:1:(n_arms-1)] ) {
        rotate( [0, 0, i*90] ) {
          translate( [core_base_diameter/2, 0, 0] ) {
          // pillar extension 
          // 
          translate( [inner_arm_length-outer_arm_width-outer_pillar_diameter/2-overlap,
                      inner_arm_thickness/2+outer_arm_length-outer_pillar_diameter/2,
                      cover_pillar_height/2+eps] )
              translate( [0,0,-eps] )
                cube( size=[outer_pillar_diameter, outer_pillar_diameter, cover_pillar_height], center=true );
          }
        }
      }
    }

    // hole for inner core pillar, so that the base-mounting nut can
    // be inserted from the top
    //
    translate( [0,0, -eps] )
      cylinder( d=core_base_diameter, h=2*cover_thickness+2*eps, center=false, $fn=fn_global );

    // for bore hoels for the tool-mounting screws
    // 
    for( i=[0:1:(n_arms-1)] ) {
      rotate( [0, 0, i*90] ) {
        translate( [core_base_diameter/2, 0, 0] ) {
          // screw bore throughout the outer pillar
          // 
            translate( [inner_arm_length-outer_arm_width-outer_pillar_diameter/2-overlap,
                        inner_arm_thickness/2+outer_arm_length-outer_pillar_diameter/2,
                        0] )
              translate( [0,0,-eps] )
                cylinder( d=outer_screw_diameter, h=cover_thickness+cover_pillar_height+2*eps, center=false, $fn=fn_global );

         }
       }
     }
  }
}
    
*/





/* **************************************************************** */
/* **************************************************************** */
/* M3207 ADAPTER PLATE                                                      */
/* **************************************************************** */
/* **************************************************************** */


// inner FT-adapter cylinder and also outer Sunrise-adapter ring
module sunrise_M3207_adapter_plate() 
{
  dd = 74.0; // M3207 outer diameter
  d2 =  9.5; // diameter of fine-screw for drill chunk (previous value of 8.5 was WRONG!)
  hh = 10.0; // total plate thickness
  eps = 0.1;
  d3 = 44.0; 
  M3 =  3.2;
  M3HEAD = 6.5;
  
  n_m3207_drill_holes = 4;
  r_m3207_drill_holes = 28.28;
  

  
  rotate( [0,180,45] )
  difference() {
    cylinder( d=dd, h=hh, center=false, $fn=100 ); 
    // translate( [0, 0, -1.5*eps] ) cylinder( d=d2, h=hh+2*eps, center=false, $fn=50 );     
    translate( [0, 0, -1.5*eps] ) cylinder( d=d3, h=hh+2*eps, center=false, $fn=50 );     
    
    // M3207 screws, four M4 drill holes
    for( i=[0:1:(n_m3207_drill_holes-1)] ) {      
      rotate( [0, 0, i*360.0/n_m3207_drill_holes] ) 
        translate( [r_m3207_drill_holes, 0, -eps ] ) 
          screw_M4( length= 10 );
    }
    
    for( i=[0:1:3] ) {
      rotate( [0,0, 45+i*90] )
        translate( [dd/2+eps, 0, hh/2] )
          rotate( [0,-90,0] ) 
            cylinder( d=M3, h=(dd)/2+eps, center=false, $fn=100 ); 
      rotate( [0,0, 45+i*90] )
        translate( [dd/2+eps, 0, hh/2] )
          rotate( [0,-90,0] ) 
            cylinder( d=M3HEAD, h=5, center=false, $fn=100 ); 
    }
  }
  rotate( [0,180,-30] ) 
    translate( [d3/2,0,0] ) 
      cylinder( d=1.9, h=hh, center=false, $fn=50 );
}



 /*
 * M3207_base_holder: central part of the M3207 adapter/holder.
 * Central cylinder with six screw holes and inner rectangular hole.
 * Nut bores all the length except the last few "hscrew" millimeters.
 */
module sunrise_M3207_base_holder( d_outer= 43.8, h=10, hscrew = 5 ) {
  core_pillar_height = h;
  M3 = 3.2;
  difference() {
      // single main block with central bore
      //
      gray05() // color( [0.9,0.6,0.9] ) 
      difference() {
        x1 = 2*core_pillar_dx + 2*core_pillar_diameter/2;
        y1 = 2*core_pillar_dy + 2*core_pillar_diameter/2;
        x2 = 2*core_pillar_dx - 2*core_pillar_diameter/2;
        y2 = 2*core_pillar_dy - 2*core_pillar_diameter/2;
        translate( [0,0,core_pillar_height/2] )
          cylinder( d=d_outer, h=core_pillar_height, center=true, $fn=100 );
          // cube( size=[x1,y1,core_pillar_height], center=true );
        
        translate( [0,0,core_pillar_height/2] )
          gray06()
            cube( size=[x2,y2,core_pillar_height+2*eps], center=true );
      }
  
      // four through holes for the screws
      // 
      hx = core_pillar_height + 2*eps;
      gray05()
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, core_pillar_height/2] ) 
            cylinder( d=core_pillar_screw_bore, h=hx, center=true, $fn=fn_global );
        }
      }

      core_nut_height  = core_pillar_height - hscrew;
      hz = core_pillar_height - core_nut_height/2;
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          // nut cavity for the mounting screw nut
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, hz] ) 
           gray05()
             cylinder( d=core_nut_diameter, h=core_nut_height+2*eps, center=true, $fn=6 );
        }
      }

      // inner bore  
      // 
      xx = 2*(core_pillar_dx - core_pillar_diameter/2);
      yy = 2*(core_pillar_dy - core_pillar_diameter/2);
      translate( [0,0,core_pillar_height/2] )
        cube( size=[xx,yy,core_pillar_height+2*eps], center=true );
      
      // diagonal groove for form-closure between base and elastic member
      //
      if (base_plate_groove_diameter > 0) {
        for( ix = [-1,1] ) {
          for( iy= [-1,1] ) {
            translate( [ix*core_pillar_dx, iy*core_pillar_dy, 0] )
              rotate( [90,0,-1*ix*iy*45] )
                cylinder( d=base_plate_groove_diameter, h=1.4*core_pillar_diameter, center=true, $fn=fn_global ); 
          }
        }
      }
      
      // side-mounting connecting screw bores
      thetas = [0,90,180,270];
      depths = [10,10,10,10];
      for( i = [0:len(thetas)-1] ) {
        rotate( [0,0,thetas[i]] )
          translate( [d_outer/2+eps,0,h/2] )
            rotate( [0,-90,0] ) 
              cylinder( d=M3, h=15, center=false, $fn=100 );
        
        rotate( [0,0,thetas[i]] )
          for( jj=[0:1:h/2] ) 
            translate( [d_outer/2-5,0,h/2+jj] )
              rotate( [0,-90,0] )
                cylinder( d=core_nut_diameter, h=4, center=false, $fn=6 );
      }
      
      // alignment slot
      rotate( [0,0,30] ) translate( [d_outer/2,0,h/2] )
        cylinder( d=2, h=h+eps, center=true, $fn=50 );
      
      
  } // core part
}



/* **************************************************************** */
/* **************************************************************** */
/* UNUSED UNUSED UNUSED                                             */
/* **************************************************************** */
/* **************************************************************** */
/* **************************************************************** */
/* **************************************************************** */
/* UNUSED UNUSED UNUSED                                             */
/* **************************************************************** */
/* **************************************************************** */
/* **************************************************************** */
/* **************************************************************** */
/* UNUSED UNUSED UNUSED                                             */
/* **************************************************************** */
/* **************************************************************** */





// UNUSED NOW
module core_pillar() {
  color( [1.0, 0.99, 0.5]) 
  difference() {
    // the main pillar
    translate( [0,0, core_pillar_height/2] )
      cube( size=[core_pillar_diameter, core_pillar_diameter, core_pillar_height], center=true );
    
    // through-hole for the mounting screws
    translate( [0,0, core_pillar_height/2] )
      cylinder( d=core_pillar_screw_bore, h=core_pillar_height+2*eps, center=true, $fn=fn_global );
    
    // nut cavity for the mounting screw nut
    translate( [0,0,core_pillar_height - core_nut_height/2] )
      cylinder( d=core_nut_diameter, h=core_nut_height+2*eps, center=true, $fn=6 );
  } 
}


/**
 * square PSOP plastic small outline package:
 * note: dimensions are different from soic package!
 */
module Atmel_ATtiny85_SOP() 
{
  body_width  = 5.25; // aka D,  min 5.13 max 5.35 no nominal given
  body_length = 5.30; // aka E1, min 5.18 max 5.40 no nominal given
  body_height = 1.7;  // aka A,  min 1.7 max 2.16 no nominal given 
  wire_length = (8.0 - 5.3)/2;  // (E-E1)/2
  wire_width  = 0.40;  // aka b, min 0.35, max 0.48 no nominal given
  wire_height = 0.25;  // aka C, min 0.15 max 0.35 no nominal given


  // main IC body 
  difference() {
    // main body
    color( [0,0,0] )
      translate( [0,0,body_height/2] )
        cube( size=[body_width, body_length, body_height], center=true );
    // pin 1 marker
    translate( [body_width/3,body_length/3,body_height] )
      cylinder( d=1, h=1, center=true, $fn=10 );
   }

   // wires
   for( i=[0:1:3] ) {
     x = (i-1.5) * 1.27;
     translate( [x, body_length/2+wire_length/2, 0.15] )
       color( [1,1,1] )
         cube( [wire_width, wire_length, wire_height], center=true );
     
     translate( [x, -body_length/2-wire_length/2, 0.15] )
       color( [1,1,1] )
         cube( [wire_width, wire_length, wire_height], center=true );
   }

  
}




module Everlight_ITR8307_cut_wires( wire_length=2.0 ) {
  Everlight_ITR8307(
    wire_length = wire_length
  );
}




/**
 * physical dimensions of the Everlight ITR8307 infrared 
 * proximity/reflex light sensor from the datasheet.
 * Dimensions in millimeters.
 *
 * (4)---- [[npn]]------(3)
 * (2)---- \[led]]------(1)
 * 
 *         |<-l->|
 */
module Everlight_ITR8307(
  length = 2.7, // +/- 0.2   body size along the wires
  width  = 3.4, // +/- 0.2   body size orthogonal to the wires
  height = 1.5, // +/- 0.2, wires are at h=0.5
  wire_length = 9.5, // 9.5;
  wire_width  = 0.5,
  wire_height = 0.15,
  wire_sep    = 1.3,
)
{
  difference() {
    // square main body
    color( [0,0,0] ) 
      translate( [0,0,height/2] )
        cube( size=[length, width, height], center=true );

    // cut-off triangle edge
    translate( [-2.2,-2.5,height/2] )
      rotate( [0,0,45] ) 
        cube( size=[3,3,height+2*eps], center=true );

    // IR receiver cutout
    translate( [0,wire_sep/2+wire_width/2,height] )
      cube( size=[2.0, 0.8, 0.6], center=true );

    // IR transmitter cutout
    translate( [0,-wire_sep/2-wire_width/2,height] )
      cube( size=[2.0, 0.8, 0.6], center=true );
  } // difference, main body

  dx = wire_length/2+length/2;
  dy = wire_sep/2 + wire_width/2;
  dz = 0.5;
  union() {
    color( [1,1,1] ) 
      translate( [ -dx, -dy, dz ] )
        cube( size=[wire_length, wire_width, wire_height], center=true );
    color( [1,1,1] ) 
      translate( [ -dx, +dy, dz ] )
        cube( size=[wire_length, wire_width, wire_height], center=true );
    color( [1,1,1] ) 
      translate( [ +dx, -dy, dz ] )
        cube( size=[wire_length, wire_width, wire_height], center=true );
    color( [1,1,1] ) 
      translate( [ +dx, +dy, dz ] )
        cube( size=[wire_length, wire_width, wire_height], center=true );
  }
}









/**
 * the rectangular PCB with on-board ITR 8037 IR sensors
 * and microcontroller(s).30
 *
 * TODO: cable "outlet"
 */
module fake_pcb( 
         pcb_diameter       = 35,  // outer diameter of sensor
         pcb_thickness       = 1,   // pcb board thickness
         core_screw_diameter = 4.1,  // central bore for M4 screw
         core_base_diameter  = 8,
         core_base_height    = 2, 

         inner_arm_length     = 10,  // dimensions of elastic inner arms (x4)
         inner_arm_height     = 10,
         inner_arm_thickness  = 4,
         inner_arm_bw_carrier = 4,   

         outer_arm_length     = 10,  // dimensions of elastic outer arms (x4)
         outer_arm_thickness     = 3,
         outer_arm_width  = 1,

         outer_pillar_height  = 12, 
         outer_pillar_diameter = 7, 
         outer_screw_diameter = 2.6,
         outer_nut_diameter   = 5.1, // (e.g. M2.5 = 5.0)
         outer_nut_height     = 5.0,   // (e.g: M2.5 = 2.0)

         itr8307_width        = 3.4,
)
{ 
  union() {
    difference() {
      // rectangular PCB circuit board plate
      //
      color( [0.4, 0.6, 0.5] )  
        intersection() {
          translate( [0,0, pcb_thickness/2] )
            cube( size=[pcb_diameter, pcb_diameter, pcb_thickness], center=true );
      
          translate( [0,0, pcb_thickness/2] ) rotate( [0,0,45] )
            cube( size=[pcb_diameter+5, pcb_diameter+5, pcb_thickness+eps], center=true );
        }
      
        
      // through screw holes for the core pillars
      // 
      z2 = pcb_thickness/2;
      h2 = pcb_thickness + eps;
      for( ix = [-1,1] ) {
        for( iy= [-1,0,1] ) {
          translate( [ix*core_pillar_dx, iy*core_pillar_dy, z2] ) 
            cylinder( d=core_pillar_screw_bore, h=h2, center=true, $fn=fn_global );
        }
      }
   
      // inner center column hole and cable pass-through
      // 
      translate( [0,0,-eps] )
        cylinder( d=core_base_diameter+eps, h=pcb_thickness+2*eps, center=false, $fn=fn_global );
    } // difference, main PCB

    // sensor offsets from center 
    ox = pcb_diameter/2 - 3; oy = 4;
    ddx = [-ox, -ox, -oy, oy, ox,  ox,  oy, -oy];
    ddy = [-oy,  oy,  ox, ox, oy, -oy, -ox, -ox]; 
    rot = [0,0, 90, 90, 180,180, 270,270];
    for( i=[0:1:len(ddx)-1] ) {
      translate( [ddx[i], ddy[i], pcb_thickness] ) 
        rotate( [0,0,rot[i]+90] ) 
          Everlight_ITR8307_cut_wires( wire_length=1.0 ); 
    }

    // ATtiny85 microcontrollers, hardcoded positions
    //
    {
      translate( [0, core_base_diameter/2 + 4.0, pcb_thickness] )
        rotate( [0,0,90] ) 
          Atmel_ATtiny85_SOP();
      translate( [ 0, -core_base_diameter/2 - 4.0, pcb_thickness] )
        rotate( [0,0,90] ) 
          Atmel_ATtiny85_SOP();
    }

  } // union 
}



/**
 * the core part of the sensor. 
 */
module elastic_cross_member(
         core_screw_diameter  = 4.1, // central bore for base-mounting screw
         core_nut_diameter    = 7.0, // outer diameter of mounting nut (e.g. M4 = 7.0)
         core_nut_height      = 4,   // height of mounting nut cavity (e.g. M4 = 3.2)
         core_base_diameter   = 8,   // outer diameter of core "pillar"
         core_pillar_height   = 10,  // total height of core "pillar",
         core_groove_diameter = 1,

         inner_arm_z_offset   = 3,
         inner_arm_length     = 10,  // dimensions of elastic inner arms (x4)
         inner_arm_height     = 10,
         inner_arm_thickness  = 4,
         inner_arm_bw_carrier = 4,   

         outer_arm_length     = 10,  // dimensions of elastic outer arms (x4)
         outer_arm_thickness     = 3,
         outer_arm_width  = 1,

         outer_pillar_height  = 12, 
         outer_pillar_diameter = 7, 
         outer_pillar_z_offset = -3,
         outer_screw_diameter = 2.6,
         outer_nut_diameter   = 5.1, // (e.g. M2.5 = 5.0)
         outer_nut_height     = 5.0,   // (e.g: M2.5 = 2.0)

         sensor_arm_y_offset  = 18,
         sensor_arm_x_offset  = -5, // x-offset from y-axis
         sensor_arm_height    = 6,
         sensor_arm_thickness = 0.5,
         sensor_plate_diameter= 6.0,
         sensor_plate_height  = 0.5,
)
{
  union() {

    // inner "core pillar" with screw bore and nut cavity
    difference() {
      // core pillar
      //
      color( [0.4, 1.0, 0.5] ) 
        cylinder( d=core_base_diameter, h=core_pillar_height, center=false, $fn=fn_global );
      
      // grooves on the bottom for form-closure with the matching cylinders
      // on the base plate central column
      translate( [0,0,0] ) rotate( [0,90,45] )
        cylinder( d=core_groove_diameter, h=core_base_diameter, center=true, $fn=fn_global );
      translate( [0,0,0] ) rotate( [0,90,135] )
        cylinder( d=core_groove_diameter, h=core_base_diameter, center=true, $fn=fn_global );
     
      // screw bore
      //
      translate( [0,0,-eps] ) {
        cylinder( d=core_screw_diameter, h=core_pillar_height+2*eps, center=false, $fn=fn_global );
      }

      // core nut cavity, drilled from top of core pillar
      // 
      translate( [0,0,core_pillar_height-core_nut_height+eps] ) {
        cylinder( d=core_nut_diameter, h=core_nut_height, center=false, $fn=6 );
      }
    }

    // four times: inner and outer armsnner and outer arms
    overlap = 1;
    n_arms = 4;
    for( i=[0:1:(n_arms-1)] ) {
      rotate( [0, 0, i*90] ) {
        translate( [core_base_diameter/2, 0, inner_arm_z_offset] ) {
          union() {
            // inner arms
            translate( [inner_arm_length/2-overlap/2, 0, inner_arm_height/2] ) 
              color( [0.4, 1.0, 0.5] ) 
                cube( size=[inner_arm_length+overlap, inner_arm_thickness, inner_arm_height], center=true );


            // outer arms
            translate( [inner_arm_length-outer_arm_width/2, outer_arm_length/2+inner_arm_thickness/2-overlap/2, outer_arm_thickness/2] ) 
              color( [0.4, 1.0, 0.5] ) 
                cube( size=[outer_arm_width, outer_arm_length+overlap, outer_arm_thickness], center=true );

            // carrier for the outer (white) pattern:
            if (true) { // if (make_bw_patterns)
            translate( [inner_arm_length-inner_arm_bw_carrier/2, inner_arm_thickness/2+outer_arm_length-inner_arm_bw_carrier/2, outer_pillar_z_offset/2-eps] ) 
              color( [1,1,1] )
                cube( size=[inner_arm_bw_carrier, inner_arm_bw_carrier, abs(outer_pillar_z_offset)], center=true );
            }

            // outer arm to outer nut pillar connection
            translate( [inner_arm_length-outer_arm_width-overlap/2,
                        inner_arm_thickness/2+outer_arm_length-outer_pillar_diameter/2,
                        outer_arm_thickness/2] ) {
              color( [0.4, 1.0, 0.5] ) 
                cube( size=[2*overlap, outer_pillar_diameter, outer_arm_thickness], center=true );
            }

            // outer nut "pillar", already translated core_base_diameter/2
            translate( [inner_arm_length-outer_arm_width-outer_pillar_diameter/2-overlap,
                        inner_arm_thickness/2+outer_arm_length-outer_pillar_diameter/2,
                        -inner_arm_z_offset] )
            difference() {
              // cubic or cylindrical outer "pillar" to the tool side
              // 
              translate( [0,0,outer_pillar_height/2+outer_pillar_z_offset] )
                color( [0.4, 1.0, 0.5] ) 
                  cube( size=[outer_pillar_diameter,outer_pillar_diameter,outer_pillar_height], center=true );
              //cylinder( d=outer_pillar_diameter, h=outer_pillar_height, center=false, $fn=fn_global );

              // screw bore throughout the outer pillar
              // 
              translate( [0,0,outer_pillar_z_offset-eps] )
                cylinder( d=outer_screw_diameter, h=outer_pillar_height+2*eps, center=false, $fn=fn_global );

              // hexagon nut cavity in the bottom of the pillar (screws goas in from top)
              //
              translate( [0,0,outer_pillar_z_offset-eps] ) rotate( [0,0,30] )
                cylinder( d=outer_nut_diameter, h=outer_nut_height+2*eps, center=false, $fn=6 );

            }
          } // union
        } // translate base_diameter/2
      } // rotate
    } // end arms (x4)
    
    if (make_thin_sensor_arms) {
    // four times: extra thin sensor arms
    sensor_arm_length= 10 ;
    for( i=[0:1:(n_arms-1)] ) {
      rotate( [0, 0, i*90] ) {
        translate( [core_base_diameter/2, 0, 0] ) { // no z-offset for the sensor plates
          union() {
            // thin arm from base column to sensor plate. x-offset needs to be smaller
            // than core_diameter/2 for this to work...
            //
            len = sensor_arm_y_offset + core_base_diameter/2;
            translate( [sensor_arm_y_offset-len/2,sensor_arm_x_offset,sensor_arm_height/2] )
              cube( size=[len, sensor_arm_thickness, sensor_arm_height], center=true );
            
            // part of the above the sensor plate
            translate( [sensor_arm_y_offset-sensor_plate_diameter/2,sensor_arm_x_offset,sensor_arm_height/2] ) 
              cube( size=[sensor_plate_diameter, sensor_arm_thickness, sensor_arm_height], center=true );

            // the actual sensor plate (for the black/white sensor)
            translate( [sensor_arm_y_offset-sensor_plate_diameter/2,sensor_arm_x_offset,sensor_plate_height/2] ) 
              cube( size=[sensor_plate_diameter,sensor_plate_diameter,sensor_plate_height], center=true );
            
            // connection to main inner arm
            //
          }
        }
      }
    }
  } // make_thin_sensor_arms


  } // elastic member union()
}
         



// xxxzzz
module lever_plate(
         core_base_diameter  = 10,  // base plate force-closure to elastic member
         core_screw_diameter = 4.1, // central bore for M4 screw
         core_groove_diameter = 1,                                    
   
         lever_plate_thickness = 2,

         inner_arm_z_offset  = 3,
         inner_arm_length    = 10,  // dimensions of elastic inner arms (x4)
         inner_arm_height    = 8,
         inner_arm_thickness = 1,
         inner_arm_bw_carrier= 4,   // diameter of black/white marker plate
)
{
  difference() {
   color( [0.99, 0.49, 0.19] ) union() {
      
      // inner core base plate for form closure with base plate from below
      // and with elastic member above.
      //
      translate( [0,0,0] )
        cylinder( d=core_base_diameter, h=lever_plate_thickness, center=false, $fn=fn_global );
      
      // two thin cylinders on top of the core cylinder for form-closure 
      // with the matching grooves in the elastic member
      // 
      translate( [0,0,lever_plate_thickness] ) rotate( [0,90,45] )
        cylinder( d=core_groove_diameter, h=core_base_diameter, center=true, $fn=fn_global );
      translate( [0,0,lever_plate_thickness] ) rotate( [0,90,135] )
        cylinder( d=core_groove_diameter, h=core_base_diameter, center=true, $fn=fn_global );
    
      // four times: extra thin sensor arms
      carrier_arm_height = 2.5;
      carrier_arm_width  = 5;
      carrier_connector_width = 2;
      extra_spacing = 1;
      overlap = 2;
      carrier_arm_length = inner_arm_length + overlap + extra_spacing; // inner_arm_length + overlap + extra_spacing;
     
      sensor_arm_width  = 2*extrusion_width;
      sensor_arm_length = 16+extra_spacing;
      sensor_arm_height = 8.5;
      
      sensor_plate_width = 5;
      sensor_plate_height = 1;
      sphere_offset = -4; // -2.0*carrier_connector_width;

     
      n_arms = 4;
      for( i=[0:1:(n_arms-1)] ) {
        rotate( [0, 0, i*90] ) {
          translate( [core_base_diameter/2, 0, 0] ) { // no z-offset for the carrier arms
            union() {
              // thick inner sensor arm
              translate( [inner_arm_length/2-overlap/2+extra_spacing/2,
                          carrier_arm_width/2 - inner_arm_thickness/2,
                          carrier_arm_height/2] )
                cube( size=[carrier_arm_length+2*eps,carrier_arm_width,carrier_arm_height], center=true );

              // connnection cube from thick arm to thin arm
              if (make_connection_cube) {
              translate( [inner_arm_length-carrier_connector_width/2,
                          -carrier_connector_width/2+eps,
                          carrier_arm_height/2] )
                cube( size=[carrier_connector_width, carrier_connector_width, carrier_arm_height], center=true );
              }

              // the flexible thin arm 
              translate( [inner_arm_length+extra_spacing-sensor_arm_length/2,
                          -carrier_connector_width-eps,
                          sensor_arm_height/2] )
                cube( size=[sensor_arm_length, sensor_arm_width, sensor_arm_height], center=true );

              // outer stabilizing wall
              translate( [inner_arm_length+extra_spacing-sensor_arm_width/2-sensor_arm_width/2,
                          carrier_arm_width/2 - inner_arm_thickness/2 - carrier_connector_width/2 + 0.5, //FUCK: what happens here???
                          sensor_arm_height/2] )
                #cube( size=[ 2*sensor_arm_width,
                             carrier_arm_width+carrier_connector_width/2+2*eps,
                             sensor_arm_height], center=true );

              // sensor plate (with black/white pattern)
              translate( [inner_arm_length-sensor_arm_length+sensor_plate_width/2 ,
                          -carrier_connector_width-sensor_plate_width/2+eps,
                          sensor_plate_height/2] )
                cube( size=[sensor_plate_width,sensor_plate_width,sensor_plate_height], center=true );

              // sensor plate to thin arm stabilizer
              translate( [inner_arm_length-sensor_arm_length+sensor_plate_width/2 ,
                          -carrier_connector_width-sensor_plate_width/4+eps,
                          sensor_arm_height/2] )
                cube( size=[sensor_arm_width,sensor_plate_width/2,sensor_arm_height], center=true );

              // connecting sphere
              translate( [inner_arm_length+sphere_offset,
                          -carrier_connector_width-eps,
                          sensor_arm_height-carrier_connector_width] )
                sphere( d=carrier_connector_width+0.2, center=true, $fn=fn_global );
                            
            }
          }
        }
      }
    } // union: positive parts
    
    union() { // the difference parts to be cut from the lever_plate
      // central screw bore
      translate( [0,0, lever_plate_thickness/2] )
        cylinder( d=core_screw_diameter, h=lever_plate_thickness+2*core_groove_diameter+2*eps, center=true, $fn=fn_global );
      
      // two thin cylinders as gooves on the bottom of the core cylinder
      // for form closure with the matching cylinders on the base plate
      // 
      translate( [0,0, 0] ) rotate( [0,90,45] )
        cylinder( d=core_groove_diameter, h=core_base_diameter, center=true, $fn=fn_global );
      translate( [0,0, 0] ) rotate( [0,90,135] )
        cylinder( d=core_groove_diameter, h=core_base_diameter, center=true, $fn=fn_global );
    }
  }
}


/** from the Openscad manual: triangular prism */
module prism(l, w, h){
       polyhedron(
               points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
               faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
               );
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





