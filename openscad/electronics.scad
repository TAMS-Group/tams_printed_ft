/** electronics.scad 
 *
 * Simple mechanical/housing models of common electronic parts.
 * All units in millimeters, use corresponding scale() to convert
 * to meters etc.
 * 
 * 2020.01.09 - add 'pro micro' and HE5B deadman switch
 * 2018.01.08 - add 'headers' and 'bores' options
 * 2018.01.05 - created (copy ATtiny etc from previous other files)
 * 
 * (c) 2018, 2020 fnh, hendrich@informatik.uni-hamburg.de
 */
 
 
 
 
eps = 0.05;
fn = 150;

// electronics_demo();


module electronics_demo() {
  d = 2.54;
  translate( [0,0,0] )       push_button_simple();
  translate( [0,10*d,0] )    Atmel_ATtiny85_SOP();
  translate( [-10*d,0,0] )   HE5B_M2_deadman_switch();

  translate( [10*d,0,0] )    arduino_mini();
  translate( [10*d,10*d,0] ) arduino_pro_mini( headers=true, bores=true );
  translate( [10*d,20*d,0] ) teensy_20( headers=false, bores=true );
  translate( [10*d,30*d,0] ) arduino_pro_micro( headers=true, bores=true );
  
  translate( [30.5*d,0,0] )  arduino_nano( headers=false, bores=true );
  translate( [30*d,10*d,0] ) teensy_32( headers=true, bores=true );
  translate( [30*d,20*d,0] ) teensy_36( headers=false, bores=true );
}
 

 
/** one of these low-end dirt-cheap push-buttons.
    Object is aligned at z=0 and centered in x and y.
    Pins are offset by 2.54mm in x and 1.4*2.54 in y.
 */
module push_button_simple() 
{
   w = 6.04;
   l = 6.04;
   h = 3.20;
   d_button = 3.30;
   h_button = 5.00 - 3.20;
   stroke = 0.2;
   l_wire = 4.50;
   
   // main body
   color( [0.5,0.5,0.5] ) 
     translate( [0,0,h/2] ) cube( size=[w,l,h], center=true );
   // actual button
   color( [0,0,0] ) 
     translate( [0,0,h-eps] ) cylinder( d=d_button, h=h_button, $fn=15, center = false );
   // four wires, 3x4 match to 2.54 hole breadboard ...
   silver() 
   for( dx=[-2.54, 2.54] ) {
     for( dy=[-1.4*2.54, +1.4*2.54] ) {
       translate( [dx,dy,h/2 - l_wire] )
         cylinder( d=1.2, h=l_wire, $fn=5, center=false );
    }
  }
}


module arduino_nano( headers=false, headers_pin_length=6, bores=true ) {
  w = 17 * 2.54;
  l = 7 * 2.54;
  h = 1.0;
  difference() {
    // main PCB
    arduino_blue() translate( [0,0,h/2] )
      cube( size=[w,l,h], center=true );
    if (bores) {
      // outer pins
      for( ix=[-7:1:7] ) {
        translate( [ix*2.54,3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [ix*2.54,-3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // SPI/programming pins
      for( iy=[-1:1:1] ) {
        translate( [8*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [7*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // outer mounting bores
      for( ix=[-8,8] ) for( iy=[-3,3] )
        translate( [ix*2.54,iy*2.54,-eps] )
          cylinder( d=1.5,h=1+2*eps,$fn=7,center=false );
    } // if bores
  }
  if (headers) {
    // outer pins
    silver()
    for( ix=[-7:1:7] ) {
      translate( [ix*2.54,3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
      translate( [ix*2.54,-3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
    }
  }
  // Atmega328
  color( [0,0,0] ) 
    translate( [-2.5*2.54, 0, h+0.5] ) 
      rotate( [0,0,45] )
        cube( size=[7,7,1], center=true );
  // mini-USB connector
  silver()
    translate( [-8*2.54,0, h+1.5] )
      cube( size=[8,8,3], center=true );
  // label
  translate( [3*2.54, 0, h+0.5] ) 
    electronics_label( "Arduino nano");
} // arduino_nano



/** original Arduino mini 05 */
module arduino_mini( headers=false, headers_pin_length=6, bores=true ) {
  w = 12 * 2.54;
  l = 7 * 2.54;
  h = 1.0;
  difference() {
    // main PCB
    arduino_blue() translate( [0,0,h/2] )
      cube( size=[w,l,h], center=true );
    if (bores) {
      // outer pins
      for( ix=[-6:1:5] ) {
        translate( [(ix+0.5)*2.54,3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [(ix+0.5)*2.54,-3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // comm/programming pins
      for( iy=[-2:1:2] ) {
        translate( [-5.5*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // analog/SPI pins 
      for( iy=[-2,-1,1,2] )
        translate( [5.5*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      for( iy=[-2,-1] )
        translate( [4.5*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
    } // if bores
  }
  if (headers) {
    // outer pins only
    silver()
    for( ix=[-6:1:5] ) {
      translate( [(ix+0.5)*2.54,3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
      translate( [(ix+0.5)*2.54,-3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
    }
  }

  // Atmega328
  color( [0,0,0] ) 
    translate( [0.5*2.54, 0, h+0.5] ) 
      cube( size=[7,7,1], center=true );
  // labels
  translate( [-3*2.54, 0, h+0.5] ) 
    electronics_label( "Arduino");
  translate( [3.8*2.54, 0, h+0.5] ) 
    electronics_label( "Mini 05");
} // arduino_mini



/** sparkfun pro mini (3.3V / 5V).
 * note: some China clones have slightly different pins,
 * especially for the SPI and extra analog pins.
 */
module arduino_pro_mini( headers=false, headers_pin_length=6, bores=true ) {
  w = 13 * 2.54;
  l = 7 * 2.54;
  h = 1.0;
  difference() {
    // main PCB
    arduino_blue() translate( [0,0,h/2] )
      cube( size=[w,l,h], center=true );
    if (bores) {
      // outer pins
      for( ix=[-5:1:6] ) {
        translate( [(ix)*2.54,3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [(ix)*2.54,-3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // comm/programming pins
      for( iy=[-3:1:2] ) {
        translate( [-6*2.54,(iy+0.5)*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // analog/SPI pins 
      for( ix=[-2,-1,1,2] )
        translate( [(ix+0.5)*2.54,2*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
    } // if bores
  }
  if (headers) {
    // outer pins only
    silver()
    for( ix=[-5:1:6] ) {
      translate( [ix*2.54,3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
      translate( [ix*2.54,-3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
    }
  }
  // Atmega328
  color( [0,0,0] ) 
    translate( [0.5*2.54, 0, h+0.5] ) 
      rotate( [0,0,45] ) 
        cube( size=[7,7,1], center=true );
  // labels
  translate( [-3.3*2.54, 0, h+0.5] ) 
    electronics_label( "Sparkfun");
  translate( [4*2.54, 0, h+0.5] ) 
    electronics_label( "Pro Mini");
} // arduino_pro_mini



module teensy_20( headers=true, headers_pin_length=6, bores=true ) {
  w = 12 * 2.54;
  l = 7 * 2.54;
  h = 1.0;
  difference() {
    // main PCB
    teensy_green() translate( [0,0,h/2] )
      cube( size=[w,l,h], center=true );
    if (bores) {
      // outer pins
      for( ix=[-6:1:5] ) {
        translate( [(ix+0.5)*2.54,3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [(ix+0.5)*2.54,-3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // extra pins
      for( iy=[-3:1:3] ) {
        translate( [5.5*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
    }
  }
  if (headers) {
    // outer pins
    silver()
    for( ix=[-6:1:5] ) {
      translate( [(ix+0.5)*2.54,3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
      translate( [(ix+0.5)*2.54,-3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
    }
  }
  // Atmega32U4
  color( [0,0,0] ) 
    translate( [1*2.54, 0, h+0.5] ) 
      cube( size=[8,8,1], center=true );
  // mini-USB connector
  silver()
    translate( [-4.6*2.54,0, h+1.5] )
      cube( size=[8,8,3], center=true );
  // label
  translate( [1*2.54, 5, h+0.5] ) 
    electronics_label( "Teensy 2.0" );
} // teensy_20


module teensy_32( bores=true, headers=false, headers_pin_length=6 ) {
  w = 14 * 2.54;
  l = 7 * 2.54;
  h = 1.0;
  difference() {
    // main PCB
    teensy_green() translate( [0,0,h/2] )
      cube( size=[w,l,h], center=true );
    if (bores) {
      // outer pins
      for( ix=[-7:1:6] ) {
        translate( [(ix+0.5)*2.54,3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [(ix+0.5)*2.54,-3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // right side pins
      for( iy=[-3:1:3] ) {
        translate( [6.5*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // extra pins
      for( ix=[-6,-4,-3,-2] ) {
        translate( [(ix+0.5)*2.54,2*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // note: bottom-side solder pins not modeled
    } // if bores
  }
  if (headers) {
    // outer pins
    silver()
    for( ix=[-7:1:6] ) {
      translate( [(ix+0.5)*2.54,3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
      translate( [(ix+0.5)*2.54,-3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
    }
  }
  // Arm Cortex-M4
  color( [0,0,0] ) 
    translate( [2*2.54, 0, h+0.5] ) 
      cube( size=[10,10,1], center=true );
  // mini-USB connector
  silver()
    translate( [-6*2.54,0, h+1.5] )
      cube( size=[8,8,3], center=true );
  // label
  translate( [-2*2.54, 0, h+0.5] ) 
    electronics_label( "Teensy 3.2" );
} // teensy_32


module teensy_36( headers=false, headers_pin_length=6, bores=true ) {
  w = 24 * 2.54;
  l = 7 * 2.54;
  h = 1.0;
  difference() {
    // main PCB
    teensy_green() translate( [0,0,h/2] )
      cube( size=[w,l,h], center=true );
    if (bores) {
      // outer pins
      for( ix=[-12:1:11] ) {
        translate( [(ix+0.5)*2.54,3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [(ix+0.5)*2.54,-3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // right side pins
      for( iy=[-3:1:3] ) {
        translate( [6.5*2.54,iy*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // extra pins
      for( ix=[-11,-9,-8,-7] ) {
        translate( [(ix+0.5)*2.54,2*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // USB host port bores
      for( ix=[-9,-8,-7,-6,-5] ) {
        translate( [(ix)*2.54,-1.9*2.54,-eps] )
          cylinder( d=1.0,h=1+2*eps,$fn=7,center=false );
      }
    } // if bores
    // note: bottom-side solder pins not modeled
  }
  // note: headers only for the "main" pins
  if (headers) {
    // outer pins
    silver()
    for( ix=[-12:1:11] ) {
      translate( [(ix+0.5)*2.54,3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
      translate( [(ix+0.5)*2.54,-3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
    }
  }

  // Arm Cortex-M4
  color( [0,0,0] ) 
    translate( [1*2.54, 0, h+0.5] ) 
      cube( size=[12,12,1], center=true );
  // mini-USB connector
  silver()
    translate( [-11*2.54,0, h+1.5] )
      cube( size=[8,8,3], center=true );
  // micro-SD card
  silver()
    translate( [9.5*2.54,0, h+0.5] )
      cube( size=[11,11,1], center=true );
  // label
  translate( [-5*2.54, 0, h+0.5] ) 
    electronics_label( "Teensy 3.6" );
} // teensy_36



/**
 * square PSOP plastic small outline package for ATtiny85.
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
  // label
  translate( [0, 0, body_height-eps] ) rotate( [0,0,180] ) 
    electronics_label( "Tiny85", letter_size=1.0 );
}


module arduino_pro_micro( headers=true, headers_pin_length=6, bores=true ) {
  w = 13 * 2.54;
  l = 7 * 2.54;
  h = 1.0;
  difference() {
    // main PCB
    teensy_green() translate( [0,0,h/2] )
      cube( size=[w,l,h], center=true );
    if (bores) {
      // outer pins
      for( ix=[-6:1:5] ) {
        translate( [(ix+1)*2.54,3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
        translate( [(ix+1)*2.54,-3.0*2.54,-eps] )
          cylinder( d=1.2,h=1+2*eps,$fn=7,center=false );
      }
      // no extra pins on the pro micro
    }
  }
  if (headers) {
    // outer pins
    silver()
    for( ix=[-6:1:5] ) {
      translate( [(ix+1)*2.54,3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
      translate( [(ix+1)*2.54,-3.0*2.54,-2/3*headers_pin_length] )
        cylinder( d=1.0,h=headers_pin_length+2*eps,$fn=10,center=false );
    }
  }
  // Atmega32U4
  color( [0,0,0] ) 
    translate( [1.5*2.54, 0, h+0.5] ) 
      cube( size=[8,8,1], center=true );
  // micro-USB connector
  silver()
    translate( [-5.8*2.54,0, h+1.5] )
      cube( size=[5,,7.4,2.5], center=true );
  // label
  translate( [1*2.54, 5, h+0.5] ) 
    electronics_label( "Pro Micro (32u4)" );
} // teensy_20



// deadman switch (off-on-off), origin at mount-point.
//& https://www.apem-idec.eu/he5b-series-474.html
//
module HE5B_M2_deadman_switch()
{
  h1 = 16.0; // total cap heigth
  d1 = 19.5; // cap diameter
  b1 =  6.0; // cap "bevel" height, approximate
  w1 =  2.0; // cap "bevel" width, approximate

  d2 = 15.5; // diameter of the bottom part 
  h2 = 15.0; // "bottom" part of the switch

  d3 = 17.5; // diameter of the screw
  h3 =  5.0; // screw 

  color( "yellow" ) {
    translate( [0,0,h1-b1] )
      minkowski() {
        cylinder( d=d1-b1, h=b1, center=false, $fn=100 );
        sphere( d=b1, center=false, $fn=100 );
      }
    cylinder( d=d1, h=h1-b1, center=false, $fn=100 );
  }

  color( "black" )
    translate( [0,0,-h2] )
      cylinder( d=d2, h=h2, center=false, $fn=fn );
  color( "black" )
    translate( [0,0,-h3-2] )
      cylinder( d=d3, h=h3, center=false, $fn=fn );
 
  // pins
  color( "silver" )
  for( x=[-4.8, 0, 4.8] ) 
    for( y=[-3.0, +3.0] )
      translate( [x,y,-h2-4.0] )
        cube( size=[2.8,0.5,8.0], center=true );;
}







module electronics_label( string, letter_size=1.5 ) {
  font = "Liberation Sans";
	// Use linear_extrude() to make the letters 3D objects as they
	// are only 2D shapes when only using text()
	linear_extrude(height = 0.2) {
		text( string, size = letter_size, font = font, halign = "center", valign = "center", $fn = 16);
	}
}


module arduino_blue() {
  color( [0,0,0.5] ) children();
}


module teensy_green() {
  color( [0,0.4,0] ) children();
}
 
 
module silver() {
   color( [0.9,0.9,0.9] ) children(); 
}

