/* utility jig for building the "2DOF sensor module" used in some 
   of the prototype sensors. No PCB provided nor needed, you may 
   want to solder-in a series resistor between the two LEDs and 
   perhaps also the pullup-resistors dimensioned for your design.
*/

include <breadboard.scad>
include <optokoppler.scad>


2DOF_sensor_module_soldering_tool();
pcb_thickness = 1.0;
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

    translate( [0,0, 0] ) breadboard( nx=4, ny=7, z=pcb_thickness, bore=1.0, fn=15, epoxy=false );
    translate( [dx1, dy1, pcb_thickness] ) rotate( [0,0,180] ) vishay_TCST1103();
    translate( [dx1, dy2, pcb_thickness] ) rotate( [0,0,0] ) vishay_TCST1103();
}
