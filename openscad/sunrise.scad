// sunrise.scad - Sunrise SAST force/torque sensor and adapter
// plates for MHI PA10-6C robot and Kuka LWR robot
//
// (c) 2016 fnh, hendrich @informatik.uni-hamburg.de
//
// parameters:
// pa10_adapter_plate_thickness [m]
// tool_adapter_plate_thickness [m]
// number_outer_connection_screws [0..n], typically 4 or 8
// outer_adapter_plate_screws [0/1]
//
// Note: the diameter of the  PA-10 tool (=J6) adapter plate should 
// best be smaller than r=45mm, so that the tool can pass inside
// the J5 Y-bracket.
//
// 2016.05.30 - fix drill adapter: inner screw diameter is 9.5mm not 8.5
// 2016.05.26 - first objects printed
// 2016.05.23 - created
// 
// TODO: implement / finalize
// 
// (c) 2016 fnh
//

eps = 0.1;
$fn = 40;



translate( [0, 0, 50] ) sunrise_M3207_tool_bohrfutter_adapter();
translate( [0, 0, 0] ) sunrise_M3207();
translate( [0, 0, -40]) sunrise_M3207_base_adapter();

#translate( [0, 0, -100] ) pa10_universal_adapter();

/*
translate( [100, 0,  0] ) screw_M6_10_countersunk();
translate( [100, 0, 20] ) screw_M6_15_countersunk();
translate( [100, 0, 40] ) screw_M6_10_countersunk_extra_rim();
translate( [80,  0,  0] ) screw_M4( length=10, head_length=3, washer_length=0 );
translate( [80,  0, 20] ) screw_M4( length=10, head_length=3, washer_length=0.8 );
translate( [120, 0,  0] ) nut_M6(); 
translate( [120, 0, 40] ) screw_M5_10_countersunk_extra_rim();
translate( [120, 0, 80] ) screw_M5();
*/



// SCAD module of the Sunrise SAST type M3207 force-torque sensor:
// outer diameter 74 mm,
// inner diameter 40 mm,
// thickness 17 mm overall, neutral axis at z=8.6,
// base thickness 11 mm, spacing 1 mm, tool-part thickness 5 mm
//
// four screw holes M4 on base and tool plate with radius 28.28 mm,
// two alignment holes (4 and 3 mm diameter) between the screw holes,
// x and y axis centered between the screw holes,
// y along (3 -> 4 mm alignment boring), 
// z pointing up from base (thin) to tool (thick) cylinder
// 
// load capacity 130N for x and y, 400N for z, 10Nm for tx, ty, tz.
//
module sunrise_M3207() 
{
  h1 = 5.0;    // thin body part
  h2 = 11.0;   // thick body part
  h3 = 1.0;    // spacing
  r1 = 74.0/2; // outer
  r2 = 40.0/2; // inner
  
  union() {
    color( [0.8,0.8,0.8] )
    translate( [0, 0, h1/2] ) {
      // thin base part (in x-y plane, oriented to -z)
      difference() {
        // outer cylinder minus 40mm diameter inner hole
        cylinder( h=h1,   r=r1, center=true, $fn=100 );
        cylinder( h=h1+1, r=r2, center=true, $fn=100 );

        // four M4 screw borings
        translate( [-20, -20, 0] ) cylinder( h=5.5, r=2.0, center=true, $fn=20 );
        translate( [-20, +20, 0] ) cylinder( h=5.5, r=2.0, center=true, $fn=20 );
        translate( [+20, -20, 0] ) cylinder( h=5.5, r=2.0, center=true, $fn=20 );
        translate( [+20, +20, 0] ) cylinder( h=5.5, r=2.0, center=true, $fn=20 );
        
        // alignment pin borings
        translate( [-28.28, 0, -h1/2-0.01] ) cylinder( h=3.5, r=1.5, center=false, $fn=20 );
        translate( [ 28.28, 0, -h1/2-0.01] ) cylinder( h=3.5, r=2.0, center=false, $fn=20 );
        
      } // thin base part
    }

    // thick tool part (in x-y plane, oriented to +z)
    translate( [0, 0, (h1+h2/2+h3)] ) {
      color( [0.8,0.8,0.8] )
      difference() {
        // outer cylinder minus 40mm diameter inner hole
        cylinder( h=h2,   r=r1, center=true, $fn=100 );
        cylinder( h=h2+1, r=r2, center=true, $fn=100 );

        // four M4 screw borings
        translate( [-20, -20, 0] ) cylinder( h=12.0, r=2.0, center=true, $fn=20 );
        translate( [-20, +20, 0] ) cylinder( h=12.0, r=2.0, center=true, $fn=20 );
        translate( [+20, -20, 0] ) cylinder( h=12.0, r=2.0, center=true, $fn=20 );
        translate( [+20, +20, 0] ) cylinder( h=12.0, r=2.0, center=true, $fn=20 );
        
        // alignment pin borings
        translate( [-28.28, 0, h2/2-3.49] ) cylinder( h=3.5, r=1.5, center=false, $fn=20 );
        translate( [ 28.28, 0, h2/2-3.49] ) cylinder( h=3.5, r=2.0, center=false, $fn=20 );
      } // thick tool part
    }
  } // M3207 union
}


module screw_M5_10_countersunk_extra_rim( length=10, extra_rim=3 ) {
  r = 2.5; rim = 1; eps=0.1; dr = 2.5; // head radius minus screw radius
  // standard rim length seems to be 1 mm on M6
  union() {
    cylinder(  r=r+dr, h=rim+extra_rim+2*eps, center=false, $fn=20 );
    translate( [0,0,rim+extra_rim+eps] )     cylinder( r1=r+dr, r2=r, h=3, center=false, $fn=20 );
    translate( [0,0,rim+extra_rim+dr] )      cylinder( r=r, h=length, center=false, $fn=20 );
  }
}


module screw_M6_10_countersunk() {
  union() {
    cylinder( d=2*6, h=1, center=false, $fn=20 );
    translate( [0,0,1] ) cylinder( r1=6, r2=3, h=3, center=false, $fn=20 );
    translate( [0,0,3] )   cylinder( d=6, h=10, center=false, $fn=20 );
  }
}

module screw_M6_10_countersunk_extra_rim( length=10, extra_rim=3 ) {
  r = 3.1; rim = 1; eps=0.1; dr = 3; // head radius minus screw radius
  // standard rim length seems to be 1 mm on M6
  union() {
    cylinder(  r=r+dr, h=rim+extra_rim+2*eps, center=false, $fn=20 );
    translate( [0,0,rim+extra_rim+eps] )     cylinder( r1=r+dr, r2=r, h=3, center=false, $fn=20 );
    translate( [0,0,rim+extra_rim+dr] )       cylinder( r=r, h=length, center=false, $fn=20 );
  }
}

module screw_M6_15_countersunk() {
  union() {
    cylinder( d=2*6, h=1, center=false, $fn=20 );
    translate( [0,0,1] ) cylinder( r1=6, r2=3, h=3, center=false, $fn=20 );
    translate( [0,0,1] )   cylinder( d=6, h=15, center=false, $fn=20 );
  }
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


module nut_M4( diameter=8.1, height=2.5 ) {
  cylinder( d=diameter, h=height, $fn=6, center=false );
}

module nut_M5( diameter=9.0, height=4.0 ) {
  cylinder( d=diameter, h=height, $fn=6, center=false );
}

module screw_M5( length=10, head_length=3, head_diameter=8.5,
                 washer_thickness=1, washer_diameter=11.0 ) {
  r = 2.6; eps=0.1;
  union() {
    cylinder( d=head_diameter, h=head_length+eps, center=false, $fn=20 ); 
    translate( [0,0,head_length] ) cylinder( r=r, h=length, center=false, $fn=20 );
    translate( [0,0,head_length] ) cylinder( d=washer_diameter, h=washer_thickness, center=false, $fn=20 );
  }
}



module nut_M6( diameter=11.4, height=5 ) {
  cylinder( d=diameter, h=height, $fn=6, center=false );
}







// adapter plate to mount tools onto a MHI PA10-6C robot,
// using four drill-holes for M6 Senkkopfschrauben.
// The plate has a circular set of 8 z-axis M4 nut drill holes 
// to connect along z-axis to other plates, and 4 radial
// M4 nut drill holes for mounting tools along x- and y-axis.
// 
module pa10_universal_adapter() 
{
  eps = 0.1;
  
  dd = 98.0;   // outer diameter of PA-10 flange 
  d1 = 63.0;   // diameter for PA-10 M6 mount screws
  d2 = 40.0;   // diameter of inner wire-through hole
 
  hh = 11.0;   // total plate thickness
  h2 =  4.0;   // M6 screws Senkkopf height
  h3 =  4.0;   // M4 screws head plus washer height
 
  M4 = 4.2;
  M5 = 5.2;
  M6 = 6.2;

  screw_inner_diameter = M6;
  screw_head_diameter = 2*M6; 
  m4_washer_diameter = 10.0;
  
  n_outer_drill_holes = 8;
  r_outer_drill_holes = 43.0;

  n_pa10_drill_holes = 4;
  r_pa10_drill_holes = 63.0/2;
  
  //color( [0.8,0.7,0.6] )
  difference() {
    // actual adapter plate
    cylinder( d=dd, h=hh, center=false, $fn=100 );
    
    
    // arm end plate
    // %translate( [0, 0, -hh-2]) cylinder(d=76, h=10, center=false, $fn=100);
    
    // inner boring for tools and wires
    translate( [0, 0, -eps] )
      cylinder( d=d2, h=hh+2*eps, center=false, $fn=100 );    

    // PA-10 countersunk screws, four M6 drill holes
    dz = 3;
    for( i=[0:1:(n_pa10_drill_holes-1)] ) {      
      rotate( [0, 0, i*360.0/n_pa10_drill_holes] ) 
        rotate( [0, 180, 0] ) translate( [d1/2, 0, -hh-dz+1 ] ) 
          screw_M6_10_countersunk_extra_rim( length=(6+5*eps), extra_rim=dz );
    }
    
    // outer screws, eight M4 full-through drill holes
    for( i=[0:1:(n_outer_drill_holes-1)] ) {      
      rotate( [0, 0, 22.5+i*360.0/n_outer_drill_holes] ) 
        translate( [r_outer_drill_holes, 0, -1.1*eps ] ) 
          cylinder( h=hh+2.1*eps, d=M5, center=false );
    }

    // outer screws, eight drill holes for M4 washers and nuts
    for( i=[0:1:(n_outer_drill_holes-1)] ) {      
      rotate( [0, 0, 22.5+i*360.0/n_outer_drill_holes] ) 
        translate( [r_outer_drill_holes, 0, -3*eps ] ) 
          rotate( [0,0,30] ) 
          // cylinder( h=h3, d=m4_washer_diameter, center=false, $fn=6 );
          nut_M5(); 
    }
  }
}



module sunrise_M3207_base_adapter() 
{
  eps = 0.1;
  
  dd = 98.0; // PA-10 outer flange / mounting plate diameter
  d2 = 40.0; // diameter of M3207 inner hole
 
  hh = 10.0; // total thickness
  h3 = 4.0;  // thickness of washer+nut holes
  
  M4 = 4.2;
  M5 = 5.2;
  M6 = 6.2;
  m4_washer_diameter = 10.0;
  
  n_outer_drill_holes = 8;
  r_outer_drill_holes = 43.0;

  n_m3207_drill_holes = 4;
  r_m3207_drill_holes = 28.28;
  
  //color( [0.8,0.7,0.6] )
  rotate( [0, 180, 45] )
  difference() {
    cylinder( d=dd, h=hh, center=false, $fn=100 );  
    
    // inner boring for tools and wires
    translate( [0, 0, -eps] )
      cylinder( d=d2, h=hh+2*eps, center=false, $fn=100 );    

    // M3207 screws, four M4 drill holes
    for( i=[0:1:(n_m3207_drill_holes-1)] ) {      
      rotate( [0, 0, i*360.0/n_m3207_drill_holes] ) 
        translate( [r_m3207_drill_holes, 0, -eps ] ) 
          cylinder( h=hh+2*eps, d=M4, center=false );
    }
    // M3207 screws, four drill holes for M4 washers
    for( i=[0:1:(n_m3207_drill_holes-1)] ) {      
      rotate( [0, 0, i*360.0/n_m3207_drill_holes] ) 
        translate( [r_m3207_drill_holes, 0, hh-h3+eps ] ) 
          cylinder( h=h3, d=m4_washer_diameter, center=false );
    }
    
    // outer screws, eight M4 full-through drill holes
    for( i=[0:1:(n_outer_drill_holes-1)] ) {      
      rotate( [0, 0, 22.5+i*360.0/n_outer_drill_holes] ) 
        translate( [r_outer_drill_holes, 0, -eps ] ) 
          cylinder( h=hh+2*eps, d=M5, center=false );
    }

    // outer screws ,eight drill holes for M4 washers and nuts
    for( i=[0:1:(n_outer_drill_holes-1)] ) {      
      rotate( [0, 0, 22.5+i*360.0/n_outer_drill_holes] ) 
        translate( [r_outer_drill_holes, 0, -eps ] ) 
          screw_M5( head_diameter = 11 );
          // cylinder( h=h3, d=m4_washer_diameter, center=false );
    }

    // visualize outer screws
    for( i=[0:1:(n_outer_drill_holes-1)] ) {      
      rotate( [0, 0, 22.5+i*360.0/n_outer_drill_holes] ) 
        translate( [r_outer_drill_holes, 0, -50-eps ] ) 
          ;// %cylinder( h=100, d=9, center=false );
    }
  } // difference
} // sunrise_M3207_base_adapter



module sunrise_M3207_tool_bohrfutter_adapter() 
{
  dd = 74.0; // M3207 outer diameter
  d2 =  9.5; // diameter of fine-screw for drill chunk (previous value of 8.5 was WRONG!)
  hh = 10.0; // total plate thickness
  eps = 0.1;
  
  n_m3207_drill_holes = 4;
  r_m3207_drill_holes = 28.28;
  
  rotate( [0,180,45] )
  difference() {
    cylinder( d=dd, h=hh, center=false, $fn=100 ); 
    translate( [0, 0, -1.5*eps] ) cylinder( d=d2, h=hh+2*eps, center=false, $fn=50 );     
    
    // M3207 screws, four M4 drill holes
    for( i=[0:1:(n_m3207_drill_holes-1)] ) {      
      rotate( [0, 0, i*360.0/n_m3207_drill_holes] ) 
        translate( [r_m3207_drill_holes, 0, -eps ] ) 
          screw_M4( length= 10 );
    }
  }
}


module Sunrise_M3207_ball_in_cup_adpater()
{

}





