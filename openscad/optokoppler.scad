/* optokoppler.scad - 3D models of different IR opto-couplers.
 *
 * 2017.10.16 - created
 * 2018.08.06 - add Everlight 1201, 8307, and 9904 devices
 *
 * (c) 2017, 2018, fnh, hendrich@informatik.uni-hamburg.de
 */


// global openscad parameters
eps = 0.01;
fn  = 100;


single_components = false;
arm_2DOF = false;
arm_3DOF = false;


if (single_components) {
translate( [0,0,0] )               vishay_TCST1103();
translate( [-8*2.54, 8*2.54, 0] )  everlight_ITR1201();
translate( [-6*2.54, 8*2.54, 0] )  rotate( [0,0,-90] )  everlight_ITR8307();
translate( [6.5*2.54,0,0] )        everlight_ITR9809();
translate( [0, 8*2.54,0] )         everlight_ITR9904( hull=true );
translate( [2.54/2,2.54/2,-1.1] )  breadboard_old( w=60, h=60, z=0.8, delta=2.54, copper=true );
}

if (arm_2DOF) {
translate( [0,0,0] )  vishay_TCST1103();
translate( [2*2.54,4*2.54,0] ) rotate( [0,0,90] ) vishay_TCST1103();
// 2-DOF lever  
translate( [0,6,7.0] ) cube( size=[2,30,3], center=true );
translate( [2.5,4*2.54,8] ) cube(size=[5,2,5], center=true );
}


if (arm_3DOF) {
translate( [0,0,0] )  vishay_TCST1103();
translate( [2*2.54,4*2.54,0] ) rotate( [0,0,90] ) vishay_TCST1103();
translate( [0,8*2.54,0] )  vishay_TCST1103();
// 3-DOF lever  
color( [0,0,1] ) translate( [0,0,10] ) cube( size=[1,30,3], center=true );
color( [1,0,0] ) translate( [2.5,4*2.54,8] ) cube( size=[5,1,5], center=true );
color( [0,1,0] ) translate( [0,16.5,8] ) cube( size=[1-eps,8,5], center=true );
// PCB 
translate( [0.5*2.54,4.5*2.54,-0.5] ) breadboard_old( w=20, h=30, z=1, delta=2.54 );
}



/**
 * model of the Everlight ITR1201 (SMD) optocouptler.
 * Orientation is "standing", so that the wires (if enabled)
 * reach to z=0. Origin is at the bottom center.
 * Dimensions are in millimeters.
 * General tolerance +/- 0.2 mm for all dimensions.
 */
module everlight_ITR1201( show_wires=true, wire_length=17.0, show_footprint=true, show_labels=true ) {
  sx = 3.40;
  sy = 2.70;
  sz = 1.50;
  rr = 1.0;
  color( [0.7,0.7,0.7] )
  difference() {
    // main body
    translate( [0, 0, sz/2] )
      cube( size=[sx, sy, sz], center=true );

    // edge cutout (simplified, actually rounded)
    translate( [-sx/2, +sy/2, sz/2] )
      rotate( [0, 0, 45] ) 
        cube( size=[0.8, 0.8, sz+2*eps], center=true );

    // transmitter and receiver cutouts
    translate( [-0.7, 0, sz-0.1] ) cube( size=[0.9, 2.0, 0.2+eps], center=true );
    translate( [+0.7, 0, sz-0.1] ) cube( size=[0.9, 2.0, 0.2+eps], center=true );
  }
  if (show_footprint) {
    dx = 0.9;
    dy = 3.2/2 + 1.0/2;
    sx = 0.9;
    sy = 1.0;
    sz = 0.1;    
    color( [0.9,0.9,0.9] ) {
      translate( [-dx, -dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
      translate( [-dx, +dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
      translate( [+dx, -dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
      translate( [+dx, +dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
    }
  }
  if (show_wires) {
    sx = 0.5;
    sy = 4.6/2 - (4.6/2 - 2.7/2);
    sz = 0.7; // actually thickness 0.2, but also bent twice
    dx = 0.9;
    dy = 2.7/2 + sy/2;
    color( [0.95,0.8,0.95] ) {
      translate( [-dx, -dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
      translate( [-dx, +dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
      translate( [+dx, -dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
      translate( [+dx, +dy, sz/2] ) cube( size=[sx,sy,sz], center=true );
    }
  }
  // labels (note: not on actual device)
  if (show_labels) {
    translate( [-0.75, -0.5, sz-0.2] ) letter( "D", letter_size=0.8 );
    translate( [-0.75, +0.5, sz-0.2] ) letter( "+", letter_size=0.8 );
    translate( [+0.75, +0.5, sz-0.2] ) letter( "E", letter_size=0.8 );
    translate( [+0.75, -0.5, sz-0.2] ) letter( "+", letter_size=0.8 );
  }
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
module everlight_ITR8307(
  show_labels = true,
  show_wires = true,
  wire_extrude = true,

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

  if (show_wires) {
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
  // labels (note: not on actual device)
  if (show_labels) rotate( [0,0,90] ) {
    sz = 1.5;
    translate( [-0.77, -0.5, sz-0.2] ) letter( "D", letter_size=0.8 );
    translate( [-0.77, +0.5, sz-0.2] ) letter( "+", letter_size=0.8 );
    translate( [+0.77, +0.5, sz-0.2] ) letter( "E", letter_size=0.8 );
    translate( [+0.77, -0.5, sz-0.2] ) letter( "+", letter_size=0.8 );
  }

}







/**
 * model of the Everlight ITR9904 optocouptler.
 * Orientation is "standing", so that the wires (if enabled)
 * are below z=0. Origin is at the bottom center.
 * Dimensions are in millimeters.
 * General tolerance +/- 0.2 mm for all dimensions.
 */
module everlight_ITR9904( show_wires=true, wire_length=17.0, show_labels=true, hull=false ) {
  sx = 11.54;
  sy = 4.2;
  sz = 6.0;
  rr = 2.0;
  color( [0.7,0.7,0.7] )
  difference() {
    // main body
    translate( [0, 0, sz/2] )
      cube( size=[sx, sy, sz], center=true );
    // edge cutout (simplified, actually rounded)
    translate( [sx/2, -sy/2, sz/2] )
      rotate( [0, 0, 45] ) 
        cube( size=[0.5, 0.5, sz+2*eps], center=true );
    if (!hull) {
      // sideways circular bore
      translate( [0, 0, 1.65] )
        rotate( [90, 0, 0] ) 
          cylinder( d=2.2, h=sy+2*eps, center=true, $fn=30 );
      // transmitter and receiver circular ports
      translate( [-1.3, 0, sz-1] ) cylinder( d=2.0, h=1+eps, center=false, $fn=30 );
      translate( [+1.3, 0, sz-1] ) cylinder( d=2.0, h=1+eps, center=false, $fn=30 );
    }
  }
  if (show_wires) {
    dx = 7.5/2;
    dy = 2.54/2;
    translate( [-dx, -dy, -wire_length] ) cylinder( d=0.5, h=wire_length, center=false, $fn=30 );
    translate( [-dx, +dy, -wire_length] ) cylinder( d=0.5, h=wire_length, center=false, $fn=30 );
    translate( [+dx, -dy, -wire_length] ) cylinder( d=0.5, h=wire_length, center=false, $fn=30 );
    translate( [+dx, +dy, -wire_length] ) cylinder( d=0.5, h=wire_length, center=false, $fn=30 );
  }
  if (hull) {
    dx = sx/2 - sy/2;
    translate( [-dx, 0, -wire_length/2] ) cube( size=[sy,sy, wire_length+2*eps], center=true );
    translate( [+dx, 0, -wire_length/2] ) cube( size=[sy,sy, wire_length+2*eps], center=true );
  }
  // labels (note: not on actual device)
  if (show_labels) {
    translate( [-7.6/2, -1.0, sz] ) letter( "+", letter_size=1.5 );
    translate( [-7.6/2, +1.0, sz] ) letter( "E", letter_size=1.5 );
    translate( [+7.6/2, +1.0, sz] ) letter( "+", letter_size=1.5 );
    translate( [+7.6/2, -1.0, sz] ) letter( "D", letter_size=1.5 );
  }
}

 
 



/**
 * model of the Vishay Semiconductor TCST1xxx optocouplers.
 * Orientation is "standing", so that the wires (if enabled)
 * are below z=0.
 * Dimensions are in millimeters.
 */
module vishay_TCST1103( 
  show_wires        = true, 
  show_optical_axis = true, 
  show_labels       = true,
  wire_length       = 7.8 )
{
  // base
  bx = 11.9;
  by = 6.3;
  bz = 3.1;
  gray07() translate( [0,0,bz/2] ) cube( size=[bx,by,bz], center=true );
  
  // emitter and detector columns
  ex = bx;
  ey = by;
  ez = 10.8 - bz;
  slotx = 3.1;
  slotz = ez;
  translate( [0,0,bz-eps] )
  difference() {
    gray07() translate( [0,0,ez/2] ) cube( size=[ex,ey,ez], center=true );
    translate( [0,0,ez/2] ) cube( size=[slotx,ey+eps,ez+eps], center=true );
    // emitter cutout
    translate( [-bx/2,0,ez] ) rotate( [0,45,0] ) 
      cube( size=[1,by+eps,1], center=true );
  }
  
  // labels
  if (show_labels) {
    translate( [-7.6/2, -1.5, 10.8] ) letter( "E" );
    translate( [-7.6/2, +1.5, 10.8] ) letter( "+" );
    translate( [+7.6/2, +1.5, 10.8] ) letter( "D" );
    translate( [+7.6/2, -1.5, 10.8] ) letter( "+" );
  }

  // semi-transparent optical axis indicator  
  if (show_optical_axis) {
    // 1103 has 1mm diameter optics, update cylinder d for 1101 and 1102...
    color( [0,0,1,0.2] ) 
      translate( [0,0,10.8-2.6] ) 
        rotate( [0,90,0] ) 
          cylinder( d=1.0, h=bx+2*0.5, $fn=30, center=true ); 
  }

  // connection leads
  if (show_wires) {
    wdx = 7.62/2; // 3*2.54, datasheet says 7.6
    wdy = 2.54/2;
    wlen = wire_length;
    translate( [-wdx, -wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
    translate( [-wdx, +wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
    translate( [+wdx, -wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
    translate( [+wdx, +wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
  }  
  
}


/**
 * 3D model of the Everlight ITR9809 opto-coupler 
 * (gabellichtschranke).
 * note the weird non-2.54 wire positions.
 */
module everlight_ITR9809( show_wires=true, show_optical_axis=true )
{
  bx = 13.0;
  by = 11.0;
  bz = 8.0;
  
  slotx = 5.0;
  sloty = 7.0;
  
  // main body with cutout
  difference() {
    gray07() translate( [0,by/2-2.85,bz/2] ) cube( size=[bx,by,bz], center=true ); 
    gray07() translate( [0,sloty/2-2.85,bz/2] ) cube( size=[slotx,sloty+eps,bz+eps], center=true );
  }

  // indicator pin
  translate( [5.1,7.27,-1.5/2] ) cylinder( d=1.5, h=1.5, center=true, $fn=20 );
  
  // lettering (visualization only, not on real device)
  translate( [-9.4/2, -1.5, 8] ) letter( "E" );
  translate( [-9.4/2, +1.5, 8] ) letter( "+" );
  translate( [+9.4/2, +1.5, 8] ) letter( "D" );
  translate( [+9.4/2, -1.5, 8] ) letter( "+" );
  
  // semi-transparent optical axis indicator  
  if (show_optical_axis) {
    color( [0,0,1,0.2] ) 
      translate( [0,0,6.0] ) 
        rotate( [0,90,0] ) 
          cylinder( d=1.0, h=bx+2*0.5, $fn=30, center=true ); 
  }
  
  // connection leads
  if (show_wires) {
    wdx = 9.45/2; // weird, off-2.54 grid
    wdy = 2.54/2;
    wlen = 6.0;
    translate( [-wdx, -wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
    translate( [-wdx, +wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
    translate( [+wdx, -wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
    translate( [+wdx, +wdy, -wlen] ) silver() cylinder( d=0.45, h=wlen, center=false );
  }
}


module breadboard_old( w=160, h=100, z=0.8, delta=2.54, d_holes = 1.3, fn = 10, copper=true ) 
{
  xmax = floor( (w/2) / delta ) * delta;
  xmin = -xmax;
  ymax = floor( (h/2) / delta ) * delta;
  ymin = -ymax;
  // echo( xmin ); echo( xmax ); echo( ymin ); echo( ymax ); echo( delta );

  difference() {  
    epoxy() cube( size=[w,h,z], center=true ); 
    for( x=[xmin:delta:xmax] ) {
      for( y=[ymin:delta:ymax] ) {
        // echo( x ); echo( y );
        translate( [x,y,0] ) cylinder( d=d_holes, h=2*z+eps, $fn = fn, center=true );
      }
    }
  }

}




module letter( l, letter_size=2.5 ) {
  font = "Liberation Sans";
	// Use linear_extrude() to make the letters 3D objects as they
	// are only 2D shapes when only using text()
	linear_extrude(height = 0.2) {
		text(l, size = letter_size, font = font, halign = "center", valign = "center", $fn = 16);
	}
}






module black() {
  color( [0, 0, 0] ) children();
}

module silver() {
  color( [0.9, 0.9, 0.9] ) children();
}

module epoxy() {
  // color( [0.1, 0.6, 0.5] ) children(); 
  color( [0.7, 0.5, 0.1] ) children(); 
}

module hartpappe() {
  color( [0.1, 0.6, 0.5] ) children(); 
}

module gray07() {
  color( [0.7, 0.7, 0.7] ) children();
}

module gray02() {
  color( [0.2, 0.2, 0.2] ) children();
}





// Weiss tactile fingers
finger_length    = 40.0; // 40 mm "useable" length, the finger is longer overall
finger_width     = 31.0; // actually 30.2mm, using 30.5 resulted in 29.5...
finger_thickness = 13.3; // 12.5; // actually 12.3, but we need to compensate for FDM "ellipses"

// dimensions of the Weiss wtg sensor with 6x14 tactels
sensor_length    = 51.5;
sensor_width     = 24.5;
sensor_thickness = 1.5;

// actual probe_tip holder dimensions
probe_thickness  = 7;  // 1 mm wall thickness for Lego and 4mm axle
probe_length     = finger_length;
wall_thickness   = 1;
bottom_thickness = 0; // e.g. 1mm, if you want a bottom 

tip_grub_screw_diameter    = 3.0; // hole for a sideways grub-screw to hold the tip
finger_grub_screw_diameter = 3.0; // two holes for sideways grub-screws to hold against the fingers



module wsg_50_probe_tip_lego() {
  difference() {
    wsg_50_probe_tip_base();
    xx = 4.5; // nominally 4.04, but adjust to correct for overextrusion
    translate( [0,0,probe_length/2] ) cube( size=[xx,xx,probe_length+2*bottom_thickness+eps], center=true );
  }
}


module wsg_50_probe_tip_4mm() {
  difference() {
    wsg_50_probe_tip_base();
    xx = 4.6; // 4.5 still too tight // nominally 4.0, but adjust to correct for overextrusion 
    translate( [0,0,probe_length/2] ) cylinder( d=xx, h=probe_length+2*bottom_thickness+eps, center=true, $fn=fn );
  }
}


module wsg_50_probe_tip_base() {
  dx = probe_thickness + 2*finger_thickness + 2*wall_thickness;
  dy = finger_width + 2*wall_thickness;
  dz = probe_length;
  fx = finger_thickness/2 + probe_thickness/2;
  fgs = finger_grub_screw_diameter;

  difference() {
    union() {
      // main block
      translate( [0,0,dz/2] ) cube( size=[dx,dy,dz], center=true );

      // optional bottom
      if (bottom_thickness > 0) {
        translate( [0,0,-bottom_thickness/2] ) cube( size=[dx,dy,bottom_thickness], center=true );
      }

      // optional stiffeners for the finger grub-screws
      if (fgs > 0) {
        ddy = finger_width/2 + wall_thickness + fgs/2;
        dz = probe_length + bottom_thickness;
        translate( [-fx,ddy,dz/2-bottom_thickness] ) cube( size=[2*fgs,fgs,dz], center=true );
        translate( [+fx,ddy,dz/2-bottom_thickness] ) cube( size=[2*fgs,fgs,dz], center=true );
      }
    }

    // two finger cut-outs
    translate( [-fx,0,dz/2] ) cube( size=[finger_thickness, finger_width, dz+eps], center=true );
    translate( [+fx,0,dz/2] ) cube( size=[finger_thickness, finger_width, dz+eps], center=true );

    // optional cut-out hole for tip grub-screw (if d>0): 
    if (tip_grub_screw_diameter > 0) {
      translate( [0,dy/4,dz/2] ) rotate( [90,0,0] ) cylinder( d=tip_grub_screw_diameter, h=dy/2+eps, center=true, $fn=fn );
    }

    // optional cut-out holes for the finger grub-screws
    if (fgs > 0) {
      ddy = finger_width/2 + wall_thickness/2 + fgs/2;
      xx = fgs + wall_thickness + eps;
      translate( [-fx,ddy,dz/2] ) rotate( [90,0,0] ) cylinder( d=fgs, h=xx, center=true, $fn=fn );
      translate( [+fx,ddy,dz/2] ) rotate( [90,0,0] ) cylinder( d=fgs, h=xx, center=true, $fn=fn );
    }
  }
}