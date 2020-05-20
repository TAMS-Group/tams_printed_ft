/* ft-spacemouse-hex-1103-teensy.ino

  "spacemouse v2"
  
  Six-axis Force/Torque sensor using Teensy 3.2 microcontroller and
  six Vishay TCST 1103 optocouplers / fork-type interrupters. The sensors 
  are placed on a circle, 60 degrees apart, with even-numbered sensors 
  intended to measure distance from the moving plate, while odd-numbered
  sensors measure plate rotation.

  This program just periodically reads the analog inputs A0 ..A5, and 
  writes the data to serial line at 115200 baud for > 100Hz samplerate.
  Combine with ROS tams_printed_ft/nodes/spacemouse.py to publish
  raw data and (once calibrated) geometry_msgs/WrenchStamped data.

  Optical feedback of device status:

  - LED blinking fast-fast-fast-fast-long: device init, 
    waiting for serial communication,
  - LED on: device operating,
  - LED fast blinking (5 Hz): overload condition (during last second),
  
  - TODO: check for hardware errors (broken cables, missing VCC, etc)
    and indicate with different blinking patterns.

  Blinking the on-board LED (~10 mA) actually disturbs the sensor readings,
  so the originally planned heartbeat signal is now disabled and replaced
  by continuous LED on during normal operation.

  Note that the Teensy A/D converter shows weird jumps in its output
  values near 50% full range, 25%/75% full range - when using the 
  internal pullup resistors. Therefore, we now use external 47kOhm resistors 
  routed from analog pins A0..A5 to the digital pins on the opposite side 
  of the Teensy board, with the digital pins driven high. To reduce noise,
  we also use a bit of exponential averaging on the analog channels.
  
  TeensyDuino source code and compiler config is here:

  ~/arduino-1.8.9/hardware/teensy/avr/cores/teensy3


  Sensor layout and wiring was done in an ad-hoc manner on the prototype; you 
  may want to order the sensors in sequence when you build your own sensor.
  
  Using a top-view of the main sensor plate, with the USB connector and the
  indicator notch at the 12-o'clock position, we have:

  1  o'clock: channel #2 (up/down)
  3  o'clock: channel #1 (sideways)
  5  o'clock: channel #4 (up/down)
  7  o'clock: channel #5 (sideways)
  9  o'clock: channel #0 (up/down)
  11 o'clock: channel #3 (sideways)

  black       - GND (common)
  matte red   - 5V IR LED supply current (need ~40mA total for 6 LEDs)


  IR emitter series resistors:

  Designed for 5V supply (either USB or regulated external supply),
  LEDs connected in pairs with common series resistor, currently
  using 180 ohm resistors, for a forward current in the middle of
  the recommended TCST1103 range (TODO: check values, original
  hardware integration documentation lost):
  
  I_LED ~ (5.0 - 2*1.2)V / 180 ohm = 14 mA


  2020.01.05 - calibrated using ATi nano17, also took rviz screencast
  2020.01.05 - testing, used (and filed) new moving plate
  2020.01.05 - add external 47kOhm pullup resistors
  2020.01.04 - add error blinking logic, use arrays throughout
  2020.01.03 - convert from older 2017/2018 sketches
  
 (c) 2017, 2018, 2020, hendrich@informatik.uni-hamburg.de  
*/
 
 
#include <stdio.h>
 
// current sensor readings 
int channel[6] = { 0, 0, 0, 0, 0, 0 };

// previous sensor readings for exponential averaging
int prev[6] = { 0, 0, 0, 0, 0, 0 };

#define ADC_12_BIT
#undef  ADC_10_BIT

#ifdef ADC_10_BIT
// hardcoded lower/upper "overload" limits, 10 bit
int lower[6] = {  50, 100,  50, 100,  50, 100 };
int upper[6] = { 950, 950, 950, 950, 900, 950 };
#else
int lower[6] = {  400,  400,  400,  400,  400,  400 };
int upper[6] = { 3800, 3800, 3800, 3800, 3800, 3800 };
#endif



int latest_overload_millis = 0; // no overload yet
int latest_blink_millis = 0;
 
int iteration;
int led = 0;
#define LED_PIN 13

char buffer[120];


void setup() {
  // we use analog pins A0..A5, but do we want the internal pullups?
  // fnh 2020.01.03: Teensy ADC actually does not work with INPUT_PULLUP,
  // which introduces strange value-"jumps" near 50% full-range.
  //

#ifdef ADC_12_BIT  
  analogReadResolution( 12 ); // up to 13, but the "jumps" only get worse
#else
  analogReadResolution( 10 ); // mostly noise-free baseline
#endif

  // analogReference( AR_INTERNAL2V23 );
  // analogReference( INTERNAL );

  // we connect the pullup resistors needed for the phototransistors
  // to digital outputs driven high; routing is straight across the
  // Teensy...
  pinMode( 11, OUTPUT ); digitalWrite( 11, HIGH );
  pinMode( 10, OUTPUT ); digitalWrite( 10, HIGH );
  pinMode(  9, OUTPUT ); digitalWrite(  9, HIGH );
  pinMode(  8, OUTPUT ); digitalWrite(  8, HIGH );
  pinMode(  7, OUTPUT ); digitalWrite(  7, HIGH );
  pinMode(  6, OUTPUT ); digitalWrite(  6, HIGH );
  
  // status LED pin, heartbeat blinking
  pinMode( LED_PIN, OUTPUT);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  // on recent Linux, somehow Teensy 3.x will not start serial communication,
  // see discussion here:
  // https://forum.pjrc.com/threads/55732-Problem-Connecting-Teensy-to-Raspberry-Pi-with-USB?p=203769#post203769
  // 
  // One other tip was to select the "Serial + Midi" profile when programming
  // to avoid the "dead" serial port...
  //
  // For now, show a 4x-fast-pause blinking during waiting for Serial. 
  // to tell the user that communication is not working yet:
  //
  while( !Serial) { // wait for serial port to connect. Needed for native USB port only
    for( int i=0; i < 4; i++ ) { // fast-fast-fast-fast-pause identifies this program
      digitalWrite( LED_PIN, 1 );
      delay( 200 ); 
      digitalWrite( LED_PIN, 0 );
      delay( 200 ); 
    }
    delay( 700 ); 
  }
} // end setup()



void loop() {
  // read the analog inputs. Reading twice to help the analog mux,
  // and performing mild exponential-averaging filter. Averaging
  // over more samples does not help.
  //
  for( int i=0; i < 6; i++ ) {
    channel[i] = analogRead( A0+i ); // first read for warmup
    channel[i] = analogRead( A0+i ); // read current sample
    channel[i] = (prev[i]/2) + (channel[i]/2); // averaging
    prev[i] = channel[i];            // remember value for next averaging
  }
  
  iteration++;
  sprintf( buffer, "D %5d  %6d %6d  %6d %6d  %6d %6d \n", 
           iteration, 
           channel[0], channel[1],
           channel[2], channel[3],
           channel[4], channel[5] );
  Serial.print( buffer ); 
  Serial.flush(); // wait until transfer finished before doing analog readings again

  // error blinking?
  bool overload = false;
  for( int i=0; i < 6; i++ ) {
    if (channel[i] < lower[i]) overload = true;
    if (channel[i] > upper[i]) overload = true;
  }

  int now = millis();
  if (overload) latest_overload_millis = now; // remember current overload
  else if ((now - latest_overload_millis) > 1000) { // reset overload flag after 1 second
    latest_overload_millis = 0;
  }

  if (overload) {
    if ((now - latest_blink_millis) > 200) {
      led = led ^ 0x1;
      digitalWrite( LED_PIN, led ); 
      latest_blink_millis = now;
    }
  }
  else {
    led = 0x1;
    digitalWrite( LED_PIN, led );
  }
    
  delay( 5 );
}
