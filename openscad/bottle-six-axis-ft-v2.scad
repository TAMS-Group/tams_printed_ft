/** bottle-six-axis-ft-v2.scad
 *
 * 2020.02.06 - add pa10 adapter (note: less fudge needed on prusa3!)
 * 2019.12.26 - add ati-nano adapter stuff
 * 2019.12.25 - adjust fin height (sensor_fins_v3)
 * 2019.12.24 - complete, test print
 * 2019.12.23 - new
 */
 
 // see end of file for our #include's...

eps = 0.01;
fn  = 150;

fdm_fudge = 0.2; // extra size for bores/holes/nuts due to shrinkage
pcb_thickness = 1.0;
wire_length = 2.0;

base_ring_height = 12; // v4: 15mm

M2NUTs = 4;    // diameter across sides
M2NUTe = 4.38; // outer diameter across edges
M2NUTm = 1.6;  // nut height
M2BORE = 2.0;

M25NUTs = 5;
M25NUTe = 5.45;
M25NUTm = 2;
M25BORE = 2.5;

M3NUTs = 5.5;
M3NUTe = 6.08;
M3NUTm = 2.4;
M3BORE = 3.0;

M4NUTs = 7.0;
M4NUTe = 7.8;
M4NUTm = 3.2;
M4BORE = 4.0;

M5NUTs = 8.0;
M5NUTe = 8.63;
M5NUTm = 4.0;
M5BORE = 5.0;


// enable one or more of these before exporting to STL:
//

make_base_ring  = true;              // bottom + spiral parts of the FT
make_upper_ring = true;              // upper (optical sensor mounting) part
make_grasp_handle = false;            // bulky hollow "grasp" cylinder

make_spare_fins = false;
make_resistor_carriers = true;      // option for upper ring
make_sensor_carriers_alone = false; // 4 sensor carriers, no optical sensors

make_ati_nano17_adapter = false;
make_ati_nano17_stand = false;

make_pa10_teleop_adapter = false;

// enable/disable optional components, check before exporting to STL:
//
use_colors = true;
explode_distance = 0; // 15; // 15; // 0 10 20 50;

show_sensor_blocks    = false; 
show_sensor_blocks_upper_ring = true;
cutout_sensor_blocks  = false;
show_sensor_carriers  = false;
show_arduino_pro_mini = true;
show_arduino_nano     = false;
make_sensor_module    = false;         // render 2DOF optical sensor module
show_ati_nano17e      = false;

 
 
// intersection() 
{
 // cube( size=[53,300,300], center=true ); 


union() { 
if (make_sensor_module) {   
  translate( [-1.5*2.54,-3*2.54,-pcb_thickness] )  2DOF_sensor_module();
}


if (make_base_ring) base_ring( base_ring_height=base_ring_height );
  

if (make_upper_ring) {
  translate( [0,0,base_ring_height+6+explode_distance] ) upper_ring(); 
}

if (make_grasp_handle) {
  gray06()
  translate( [0,0,-explode_distance] ) grasp_handle();
}


if (make_spare_fins) { spare_sensor_fins(); } 


if (make_sensor_carriers_alone) {
  for( theta=[0,90,180,270] ) {
    rotate( [0,0,theta] ) 
      translate( [20,0,1.3*eps] ) { 
        // 2DOF_sensor_carrier_mounting_block();
        2DOF_sensor_carrier( show_sensor=false );
      } 
  }
}


if (make_ati_nano17_adapter) {
  color( "green" )
   translate( [0,0,0] ) ati_nano17_adapter();
}


if (make_ati_nano17_stand) { // aligned at z=0
  color( "green" )
  translate( [0,0,-14.5] ) ati_nano17_stand(); 
}


if (show_ati_nano17e) {
  translate( [0,0,0] ) ati_nano17e(); 
}

if (make_pa10_teleop_adapter) {
  color( "lightblue" )
   translate( [0,0,0] ) pa10_teleop_adapter( d_outer=70-0.6);
}




}} // union; intersection



/**
 * the upper part of the F/T sensor, consisting of:
 * and inner ring to be glued onto the base_ring,
 * mounting blocks for the optical sensor modules ("carriers"),
 * hex nut screw holes for the mounting screws
 * a carrier for the Arduino microcontroller
 */
module upper_ring( upper_ring_height = 7)
{
  hh = upper_ring_height;// inner ring height
  hh1 = hh+1;
  d_outer = 90.0; // was: 77.5; 
  
  // gray08() 
  difference() { // inner ring with axis alignment markers
    union() {

      // main carrier ring
      // 
      translate( [0,0,-1] )
        ring( d_inner=70-eps, d_outer=d_outer,  h=hh+1, center=false, fn=100 );

      
      for( theta=[0,90,180,270] ) {
        rotate( [0,0,theta] ) 
          translate( [46.5,0,-2] ) 
            2DOF_sensor_carrier_mounting_block();
      }
      
      // makeshift arduino mount
      //
      dx = 11;
      rotate( [0,0,30] ) 
        translate( [77.5/2+dx/2-eps, 0, hh1/2-1] )
        cube( size=[dx,2,hh1], center=true );
      rotate( [0,0,60] ) 
        translate( [77.5/2+dx/2-eps, 0, hh1/2-1] )
        cube( size=[dx,2,hh1], center=true );
      
      // resistor carriers...
      if (make_resistor_carriers) {
        for( theta=[0,90,180,270] ) {
         rotate( [0,0,theta+45] ) 
           translate( [42,0,hh/2] ) 
             cube( size=[12,26,hh], center=true );
        }   
      }
      
    } // union

    // inner ring touching the bottle
    // 
    translate( [0,0,-1-eps] )
      ring( d_inner=70-2*eps, d_outer=73.2,  h=1.1, center=false, fn=100 );

    // subtract inner cylinder to get rid of the protruding inner parts of the mounting blocks
    //
    translate( [0,0,-1-eps] ) 
      cylinder( d=70+1.5*eps, h=hh1+3*eps, center=false, $fn=fn );
    
    // axis alignment markers (notches) at +/-x and +/- y
    // 
    for( theta=[0,90,180,270] ) { 
      rotate( [0,0,theta] ) 
        translate( [70/2,0,-eps] )
          cylinder( d=1, h=hh, center=false, $fn=10 );
    }
    
    // sensor mounting block core cutouts
    for( theta=[0,90,180,270] ) { 
      rotate( [0,0,theta] ) 
        translate( [44+eps,0,hh1/2-1] )
          cube( [10,18-eps,hh1+5*eps], center=true );
    }

    // hex nut cut-outs
    m3nut = M25NUTe + 0.2;
    translate( [0,12,3] ) rotate( [0,90,0] ) rotate( [0,0,30] ) 
      cylinder( d=m3nut, h=79, center=true, $fn=6 );
    translate( [0,-12,3] ) rotate( [0,90,0] ) rotate( [0,0,30] ) 
      cylinder( d=m3nut, h=79, center=true, $fn=6 );
    translate( [12,0,3] ) rotate( [0,90,90] ) rotate( [0,0,30] ) 
      cylinder( d=m3nut, h=79, center=true, $fn=6 );
    translate( [-12,0,3] ) rotate( [0,90,90] ) rotate( [0,0,30] ) 
      cylinder( d=m3nut, h=79, center=true, $fn=6 );

    // screw cut-outs 
    translate( [0,12,3] ) rotate( [0,90,0] ) 
      cylinder( d=M3BORE, h=d_outer+5, center=true, $fn=30 );
    translate( [0,-12,3] ) rotate( [0,90,0] ) 
      cylinder( d=M3BORE, h=d_outer+5, center=true, $fn=30 );
    translate( [12,0,3] ) rotate( [0,90,90] ) 
      cylinder( d=M3BORE, h=d_outer+5, center=true, $fn=30 );
    translate( [-12,0,3] ) rotate( [0,90,90] ) 
      cylinder( d=M3BORE, h=d_outer+5, center=true, $fn=30 );
    

  } // difference (rings minus hex-nut cutouts)

  if (show_arduino_nano) {
    translate( [33,35,hh1] )
      rotate( [90,0,135] )
      // rotate( [0,0,135] )
        arduino_nano();
  }

  if (show_arduino_pro_mini) {
    translate( [32+0.7*explode_distance,32+0.7*explode_distance,hh1] )
      rotate( [0,0,-45] )
        arduino_pro_mini();
  }

if (show_sensor_blocks_upper_ring) {
  difference() {
    for( theta=[0,90,180,270] ) {
      rotate( [0,0,theta] ) {
        translate( [46.5+explode_distance,0,8+explode_distance] )  
          rotate( [0,180,180] ) 
            2DOF_sensor_carrier();
      }
    }
    if (cutout_sensor_blocks) {
      translate( [0,0,-0.333+explode_distance] ) 
        ring( d_inner=96, d_outer=110, h=40, center=false, fn=fn ); 
    }
  } // difference
} // if make_sensor_blocks

  
} // upper_ring





/**
 * the bottom part of the F/T sensor, consisting of:
 * - the inner ring touching/holding the bottle
 * - the elastic spring connecting inner and outer rings
 * - the outer ring for grasping the sensofalser, with "fins"
 */
module base_ring( base_ring_height=15 ) 
{
gray08() {
difference() { // inner ring with axis alignment markers
  ring( d_inner=70, d_outer=73, h=base_ring_height+6, center=false, fn=100 );
  for( theta=[0,90,180,270] ) {
    rotate( [0,0,theta] ) 
      translate( [70/2,0,-eps] )
        cylinder( d=1, h=base_ring_height+7, center=false, $fn=10 );
  }
}  
  
ring( d_inner=73, d_outer=76,  h=base_ring_height, center=false, fn=100 );
  
difference() { // outer ring with axis markers
  ring( d_inner=95, d_outer=114, h=base_ring_height, center=false, fn=100 );
  for( theta=[0,90,180,270] ) {
    rotate( [0,0,theta] ) 
      translate( [114/2,0,-eps] )
        cylinder( d=1, h=base_ring_height+1, center=false, $fn=10 );
  }
}  

ix = 3.8; iy = 9;  iz = base_ring_height;  // inner spring mounting block
ax = 5.48; ay = 8;  az = base_ring_height;  // outer spring mounting block
for( theta=[0,90,180,270] ) {
  rotate( [0,0,theta] ) 
    translate( [76/2+ix/2-1,0,az/2] )  
      rotate( [0,0,6] )
        cube( size=[ix,iy,iz], center = true );

  rotate( [0,0,theta-30] ) 
    translate( [95/2-ax/2+1,0,az/2] ) 
      rotate( [0,0,2] )
        cube( size=[ax,ay,az], center = true );
}
} // gray

red06()
// for( theta=[0,90,180,270] ) {
for( theta=[0,90,180,270] ) {
  rotate( [0,0,theta] ) 
    translate( [1,-2.7,1.5*eps] ) 
      rotate( [0,0,-4] ) 
        spring_segment( d_inner=82, d_outer=88, h=base_ring_height-3*eps );
}

fx =15; fy = 3; fz = base_ring_height; // fin mounting blocks
gray04() 
for( theta=[0,90,180,270] ) {
  rotate( [0,0,theta] ) 
    translate( [0,47.3,fz/2+eps] )  
        cube( size=[fx,fy,fz], center = true );
}

gray04()
for( theta=[0,90,180,270] ) { // four sensor fins
  rotate( [0,0,theta] ) 
    translate( [0,46.4,base_ring_height] ) 
       sensor_fin();
}

// rings with increasing outer diameter, finishing at h=15mm
gray06()
for( i=[0.5:0.5:4.5] ) {
  translate( [0,0,base_ring_height+i] )
    ring( d_inner=72, d_outer=72+1.2*i, h=0.5+eps, center=false, fn=fn );
}

if (show_sensor_blocks) {
  difference() {
    for( theta=[0,90,180,270] ) {
      rotate( [0,0,theta] ) {
        translate( [46.5+explode_distance,0,24+explode_distance] )  
          rotate( [0,180,180] ) 
            2DOF_sensor_carrier();
      }
    }
    if (cutout_sensor_blocks) {
      translate( [0,0,-0.333+explode_distance] ) 
        ring( d_inner=96, d_outer=110, h=40, center=false, fn=fn ); 
    }
  } // difference
} // if make_sensor_blocks

} // end module base_ring



module spring_segment( d_inner=85, d_outer=90, h=10, fn=fn )
{
  intersection() {
    ring( d_inner=d_inner, d_outer=d_outer, h=h, center=false, fn=fn );

    union() {  
      cube( size=[130,130+eps,h+1], center=false );
      rotate( [0,0,-20] ) 
        cube( size=[130,130+eps,h+1], center=false );
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



/**
 * the outer base ring (diameter ~114) is a bit too large
 * for test persons with small hands. Also, we want the most
 * natural bottle grasp possible, which means an outer grasping
 * ring just large enough so that the bottle inside can move
 * freely. The conical adapter piece uses 45 degrees, which
 * should be printable on common FDM printers.
 * Origin is at the top (z=0), xy centered.
 */
module grasp_handle(
  d_inner = 70+6, 
  h_inner = 70,        // actual grasp handle height, >=25
  d_outer = 114,
  d_outer_inner = 95, 
  h_outer = 2.0,
  wall_thickness = 2.5,
  h_bottom = 4.0,
)
{
  // upper outer ring that connects (e.g. glued) to the base ring
  //
  translate( [0,0,-h_outer] ) 
    ring( d_inner=d_outer_inner, d_outer=d_outer, h=h_outer, fn=fn );

  // calculate required dimensions for the conical and the grasping rings
  //  
  d_grasp = d_inner + 2*wall_thickness; 
  d_delta = d_outer_inner - d_inner; // e.g. 95.0 -> 74.0
  h_cone  = d_delta / 2; 
  
  translate( [0,0,-h_outer-h_cone+eps] )
    difference() {
      cylinder( d1=d_inner + 2*wall_thickness,
                d2=d_outer_inner + 2*wall_thickness,
                 h=h_cone, center=false, $fn=fn );
      translate( [0,0,-eps] )
        cylinder( d1=d_inner, d2=d_outer_inner, h=h_cone+2*eps, center=false, $fn=fn );
  }


  // cylindrical grasp handle part, d >> bottle diameter + margin
  //
  z2 = -h_outer - h_cone - h_inner + 2*eps;
  translate( [0,0,z2] ) 
    ring( d_inner=d_inner, d_outer=d_grasp, h=h_inner, fn=fn );
  
  // one bottom ring for haptic "border" identification
  // 
  z3 = z2 - h_bottom;
//  translate( [0,0,z3] )
//    ring( d_inner=d_inner, d_outer=d_inner + 6*wall_thickness, h=h_bottom, 
//          center=false, $fn=fn );
  
  
  translate( [0,0,z3] )
    difference() {
      cylinder( d2=d_grasp, 
                d1=d_inner + 6*wall_thickness,
                 h=h_bottom, center=false, $fn=fn );
      translate( [0,0,-eps] )
        cylinder( d1=d_inner, d2=d_inner, h=h_bottom+2*eps, center=false, $fn=fn );
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
  xx = 14, yy = 16, hh =10,  // main carrier block (v2 had hh=10)
  xx2 = 3.0, yy2 = 8.0,       // side mounting screw blocks
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
    // baseplate 
    translate( [0,0,hh/2] ) 
      cube( size=[xx,yy,hh-eps], center=true );
    
    // central slot cutout, slot width 3.1, we use 6.2 here
    translate( [0,0,(hh)/2 ] )
      cube( size=[6.2, 3*2.54 + 6.3, hh+2*eps], center=true );
    
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
  
  // M3 (inbus or slotted) screw holder; cutout and bore with
  // oversize to allow yz-axis adjustment of the module.
  //
  dyy = yy/2 + yy2/2;
  difference() {
    translate( [xx2/2, dyy, hh/2] )
      cube( size=[xx2, yy2, hh], center=true );
    translate( [xx2/2, dyy, hh/2] )
      rotate( [0,90,0] ) 
        cylinder( d=4.5, h=xx2+eps, center=true, $fn=20 );
    translate( [xx2/2, dyy, hh/2-1] )
      rotate( [0,90,0] ) 
        cylinder( d=4.5, h=xx2+eps, center=true, $fn=20 );
    translate( [xx2/2, dyy, hh/2+1] )
      rotate( [0,90,0] ) 
        cylinder( d=4.5, h=xx2+eps, center=true, $fn=20 );
  }
  difference() {
    translate( [xx2/2, -dyy, hh/2] )
      cube( size=[xx2, yy2, hh], center=true );

    translate( [xx2/2, -dyy, hh/2-1] )
      rotate( [0,90,0] ) 
        cylinder( d=4.5, h=xx2+eps, center=true, $fn=20 );
    translate( [xx2/2, -dyy, hh/2+1] )
      rotate( [0,90,0] ) 
        cylinder( d=4.5, h=xx2+eps, center=true, $fn=20 );
    translate( [xx2/2, -dyy, hh/2] )
      rotate( [0,90,0] ) 
        cylinder( d=4.5, h=xx2+eps, center=true, $fn=20 );
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

    translate( [0,0,0] ) breadboard( nx=4, ny=8, z=pcb_thickness, bore=1.0, fn=15, epoxy=false );
    translate( [dx1, dy1, pcb_thickness] ) rotate( [0,0,180] ) vishay_TCST1103( show_optical_axis=show_optical_axis, wire_length=wire_length );
    translate( [dx1, dy2, pcb_thickness] ) rotate( [0,0,0] ) vishay_TCST1103( show_optical_axis=show_optical_axis, wire_length=wire_length );
}    
    
    
/** 
 * simple adapter to mount the sensor (w/o grasp handle)
 * to a ATi nano17 F/T sensor for testing and calibration.
 *
 * Consists of a mounting plate with nano17 screw bores,
 * and an inner ring that in turn fits snugly into the bottom 
 * ring of the bottle F/T sensor.
 *
 * Note that the cable of the nano17e (radial cable connector) 
 * only just fits into the 74 mm inner diameter of the grasp handle...
 * If necessary, a grasp handle with cable cutout could be used.
 */
module ati_nano17_adapter( 
  d_outer = 70.0 - 1.0, // 70-0.3 did not fit: too big
  h_outer = 10, h_plate = 4,
  inbus=false, inbus_head_height=2 
) 
{
  d_inner = d_outer - 4.0;
  
  m2          = 2.0;    // diameter of M2 screw thread
  m2hex       = 3.0;    // diameter of M2 hex screw head
  rs          = 12.5/2; // radius of screws
  h           = h_plate;

  difference() {
    union() {
      // main bottom plate
      cylinder( d=d_inner+eps, h=h_plate, center=false, $fn=fn );

      // outer pass ring to match inner diameter of the bottle ft
      //
      ring( d_outer=d_outer, d_inner=d_inner, h = h_outer, center=false, $fn=fn );
     
      // adjustment screw blocks
      for( phi=[0,120,240] ) {
        rotate( [0,0,phi] )
          translate( [d_inner/2-5,0,h_outer/2+eps] )
            cube( size=[10,10,h_outer], center=true );
      } 
    }
    
    // ATi nano17 tool-side screw bores
      // three M2 screw holes 
    //
    for( phi=[0,120,240] ) {
      rotate( [0,0,phi+60] ) 
        translate( [rs, 0, -eps] )
          cylinder( d=m2, h=h+2*eps, center=false );
    } 

    // cavities for three M2 inbus screw heads (if requested)
    //
    for( phi=[0,120,240] ) {
      rotate( [0,0,phi+60] ) 
        translate( [rs, 0, h-inbus_head_height] )
          cylinder( d=m2hex, h=inbus_head_height+eps, center=false );
    } 
    
    // cavitities for three M3 inbus tightening screws (optional)
    for( phi=[0,120,240] ) {
      rotate( [0,0,phi] )
        translate( [d_inner/2-15,0,h_outer-3] )
          rotate( [0,90,0] ) 
            cylinder( d=3.1, h=20, center=false, $fn=fn );
    } 
    
  }
}





/**
 * A stand/holder for the ATi nano17e->adapter->bottle ft setup;
 * high enough to allow fixing everything on a bench vise even
 * with the bottle ft grasp handle mounted.
 * Centered in x and y, origin at top (z=0).
 *
 * ATi nano17e base-side mounting plate with three M2 screws,
 * oriented along ATi reference coordinate axes.
 */
module ati_nano17_stand( 
  h=2, inbus=false, inbus_head_height=1,
  d_plate = 30,
  h_plate = 2,
  h_tower = 70,
  h_solid_base = 20,
  wall_thickness = 3,
) 
{

  m2          = 2.0;    // diameter of M2 screw thread
  m2hex       = 4.5;    // diameter of M2 hex screw head (real is: 3.8mm)
  rs          = 12.5/2; // radius of screws

  difference() {
    // top plate
    translate( [0,0,-h_plate/2] )
      cube( size=[d_plate, d_plate, h_plate], center=true );

    // three M2 screw holes 
    //
    for( phi=[0,120,240] ) {
      rotate( [0,0,phi+90+5] ) 
        translate( [rs, 0, -h_plate-eps] )
          cylinder( d=m2, h=h_plate+2*eps, center=false );
    } 

    // cavities for three M2 inbus screw heads (if requested)
    //
    for( phi=[0,120,240] ) {
      rotate( [0,0,phi+90+5] ) 
        translate( [rs, 0, -h_plate-1.5*eps] )
          cylinder( d=m2hex, h=inbus_head_height+eps, center=false );
    } 
  }
  
  // side walls
  translate( [-d_plate/2+wall_thickness/2, 0, -h_tower/2] )
    cube( size=[wall_thickness, d_plate, h_tower], center=true );
  translate( [+d_plate/2-wall_thickness/2, 0, -h_tower/2] )
    cube( size=[wall_thickness, d_plate, h_tower], center=true );
  
  // solid base
  translate( [0, 0, -h_tower -h_solid_base/2] )
    cube( size=[d_plate, d_plate, h_solid_base], center=true );
  
  // some strutting
  translate( [0, -d_plate/2+wall_thickness/2, -h_tower/2] )
    cube( size=[d_plate, wall_thickness, d_plate], center=true );
}




/** 
 * simple adapter to mount the sensor (w/o grasp handle)
 * to our PA10-6C robot for testing and f/t teleop demo.
 *
 * Consists of a mounting plate, mounting cylinder,
 * and an inner ring that in turn fits snugly into the bottom 
 * ring of the bottle F/T sensor.
 */
module pa10_teleop_adapter( 
  d_outer = 70.0 - 1.0, // 70-0.3 did not fit: too big
  h_outer = 12, h_plate = 4,
  h_tower = 30, d_tower = 20,
  h_tower2 = 15, d_tower2 = 10.0,
  inbus=false, inbus_head_height=2 
) 
{
  d_inner = d_outer - 4.0;
  
  m2          = 2.0;    // diameter of M2 screw thread
  m2hex       = 3.0;    // diameter of M2 hex screw head
  rs          = 12.5/2; // radius of screws
  h           = h_plate;

  difference() {
    union() {
      // main bottom plate
      cylinder( d=d_inner+eps, h=h_plate, center=false, $fn=500 );

      // outer pass ring to match inner diameter of the bottle ft
      //
      ring( d_outer=d_outer, d_inner=d_inner, h = h_outer, center=false, fn=500 );
     
      // adjustment screw blocks
      l_block = 5;
      for( phi=[0,120,240] ) {
        rotate( [0,0,phi] )
          translate( [d_inner/2-l_block/2,0,h_outer/2+eps] )
            cube( size=[l_block,10,h_outer], center=true );
      } 
    }
    
    // cavitities for three M3 inbus tightening screws (optional)
    for( phi=[0,120,240] ) {
      rotate( [0,0,phi] )
        translate( [d_inner/2-15,0,h_outer-3] )
          rotate( [0,90,0] ) 
            cylinder( d=3.1, h=20, center=false, $fn=fn );
    } 
  }
  
  // main "connecting" tower
  h1 = h_plate-eps;
  translate( [0,0,h1] )
    cylinder( d=d_tower, h=h_tower, center=false, $fn=200 );
  
  // tower2 to insert into PA10-6C drill-chuck mount
  h2 = h_plate + h_tower - eps;
  translate( [0,0,h2] )
    cylinder( d=d_tower2, h=h_tower2, center=false, $fn=200 );
  
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
include <ati-nano17e.scad>
