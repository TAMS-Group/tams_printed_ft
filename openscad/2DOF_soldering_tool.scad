/* utility jig for building the "2DOF sensor module" used in some 
   of the prototype sensors. No PCB provided, using a standard 0.1" 
   breadboard works fine. You can, of course, also design a small
   PCB for this, or directly design a larger PCB that carriers 
   multiple sensors.
   
   Insert the TCSTs carefully, then solder the pins to mechanically
   fix the LED and phototransistor inside their housing. Then, you
   might also want to solder-in a series resistor between the two LEDs 
   and perhaps also the pullup-resistors dimensioned for your design.
   Sensor layout can be either parallel or opposite (as shown here,
   in an attempt to avoid direct line-of-sight from an emitter LED
   to the "wrong" phototransistor).
   
   As shown, the PCB/breadboard is one unit ("hole") wider than
   necessary, to provide mechanical contact to the carrier on the
   outside, and also to give you extract soldering positions for
   the external cable. Typical wire would have VCC + GND + 2x signal
   
   Once soldered, carefully extract the two sensors from the jig
   and insert into the corresponding sensor carrier part.
*/

include <breadboard.scad>
include <optokoppler.scad>


pcb_thickness = 1.0;
corner_cutout_diameter = 0.5;

// jfig for building precisely aligned 2-DOF sensor modules
2DOF_sensor_module_soldering_tool();

// for visualization, comment-out when exporting for 3D printing
translate( [30,0,10] ) 2DOF_sensor_module();



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
     // main body
     translate( [0,0,ez/2] ) cube( size=[40,40,ez], center=true);
     // cutouts for the TCST 1103 couplers
     translate( [0,-1.5*2.54,ez/2] ) cube( size=[bx+eps,by+eps,ez+eps], center=true );
     translate( [0,+1.5*2.54,ez/2] ) cube( size=[bx+eps,by+eps,ez+eps], center=true );
     // extra corner cutouts for reliable printing on FDM/FFF machines
     for( dx=[ -bx/2, +bx/2 ] )
       for( dy=[ -by-0.25*2.54, -0.25*2.54, 0.25*2.54, 0.25*2.54+by] )
         translate( [dx,dy,ez/2] )
           cylinder( d=corner_cutout_diameter, h=ez+3*eps, center=true, $fn=30 );
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

    translate( [0,0, 0] ) breadboard( nx=4, ny=7, z=pcb_thickness, bore=1.0, fn=15, epoxy=false );
    translate( [dx1, dy1, pcb_thickness] ) rotate( [0,0,180] ) vishay_TCST1103();
    translate( [dx1, dy2, pcb_thickness] ) rotate( [0,0,0] ) vishay_TCST1103();
}
