/* ft-bottle-ft.ino

  Firmware for the "bottle-ft" six-axis force/torque sensor based on
  eight Vishay-TCST1103 optocouplers/photo-interruptor sensors. Read
  the sensors periodically and write the readings to serial or USB.

  The current prototype uses an Arduino Pro-Mini 3.3V 8MHz microcontroller
  and communicates via RS232 and an FTDI usb-serial adapter with the 
  host PC. The point is that the cable is more flexible than a standard
  USB cable (as used on most of our other wired sensors).

  On our interruptor-based 6-axis force-torque sensor designs, small fins 
  are used to block the light in the photo-interruptors, proportional to
  mechanical deflection, and therefore, to force/torque. The micro-
  controller only outputs raw sensor readings, scaling and calibration
  has to be done on the host computer. While the mechanical structure
  of the sensor tries to decouple axis-parallel forces and torques,
  the actual decoupling has to be done in software on the host, also.

  On the "bottle-ft" six axis sensor, the photo-interruptors are placed
  in pairs, sharing a common fin. Here, the thin higher part blocks
  one sensor from left-to-right, while the lower broader part blocks
  the other sensor from below. Each sensor pair has the IR LEDs 
  connected in series, and the phototransistors share common GND,
  so that four cables are needed for every two sensors:
  
  Current cable encoding (soldered with MiniPro 8MHz 3.3V):
  green -  positive supply (VSS) for the LEDs, ~2.4V. The design
           has 160 Ohm series resistors, whiche gives 
           I = U / R = (3.3V - 2*1.2V) / 160 Ohm = 6 mA per LED
  red    - phototransistor 1 (D connected to GND, so needs pullup)
  orange - phototransistor 1 (D connected to GND, so needs pullup)
  black  - GND

  Soldering the pullup-resistors was done "randomly", choosing local
  cable routing instead of considering the global picture. Channel
  assignments on that prototype is:

  "red" sensor:    up/down: #7, sideways #6   (near Arduino pro mini)
  "green" sensor:  up/down: #1, sideways #0
  "blue" sensor:   up/down: #5, sideways #2
  "yellow" sensor: up/down: #3, sideways #4.

  Range should be approximately full-voltage swing (AD counts 100..1023)
  for all sensors; adjust the sensor position mechanically as necessary.

  Suitable resistors:

  LEDs: R = (VCC - 2*1.2V) / I_desired, e.g. R=160 Ohm at 3.3V and 6 mA.
  pull-up resistors: depends on forward LED current and sensor "hacking".
  
  With semi-transparent foil glued into the sensors, the light is dimmed,
  and external pullup resistors are required to maintain full voltage swing.
  Our current design has R = 240 kOhm with "matte" paper glued onto both
  sides of the sensor, using the Arduino pro-mini at 3.3V supply.

  FTDI Basic adapter: (Sparkfun, USB-RS232 converter chip)

  The six-pin connector on the FTDI Basic directly connects to the 6-pin
  programming header on the Arduino Pro-mini. Be sure to select (or solder)
  the correct voltage, either 3.3V or 5V. Note that ALL SIX pins seem to
  be required for programming. On the Pro-Mini, two pins are marked "GND"
  (and are connected), but on the FTDI one pin is CTS and only the outer
  pin actually carries GND supply. 

  For FTDI Basic serial communication, only VCC (red), GND (black), and
  TX are needed; RX is needed for bidirectional communication.
  
  Note: we currently don't pause or sleep at all in the main loop; as
  reading all 8 ADC channels already takes some time. Still, we reach 
  about 100Hz sample rate. Every sensor channel is read twice, which
  seems to be enough to stabilize ADC input after switching channels.
  
  We might interleave part of the serial communication (the data from the
  previously read channel) while initializing one channel and sampling
  the ADC converter several times. Not sure if this is worthwile.
  
  (c) 2017, 2019 , hendrich@informatik.uni-hamburg.de  
 */
 
 #include <stdio.h>
 
 
 int channel0, channel1, channel2, channel3;
 int channel4, channel5, channel6, channel7;
 int iteration;
 int led = 0;
 char buffer[120];



void setup() {
  // we use analog pins A0..A7, and we need external pullup resistors
  // for full ADC voltage swing.
  //
  pinMode( A0, INPUT );
  pinMode( A1, INPUT );
  pinMode( A2, INPUT );
  pinMode( A3, INPUT );
  pinMode( A4, INPUT );
  pinMode( A5, INPUT );
  pinMode( A6, INPUT );
  pinMode( A7, INPUT );
  
  // LED pin, optional blinking for visual feedback
  ///
  pinMode(13, OUTPUT);

  // initialize serial communication: note timing errors at higher baudrates
  // due to slow 8MHz clock and imprecise resonators instead of quartz.
  // Forums recommend baudrates that are clean dividers of clock rate, e.g. 250000
  // Serial.begin(115200); // works for arduino->pc, but has trouble pc->arduino
  // 
  Serial.begin( 250000 );
  delay( 1000 );
}


void loop() {
  // read the analog inputs (twice for stable values after amux switching).
  // Cannot do more filtering due to slow 8MHz clock speed on pro mini 3v3.
  //
  channel0 = analogRead(A0);
  channel0 = analogRead(A0);
  channel1 = analogRead(A1);
  channel1 = analogRead(A1);
  channel2 = analogRead(A2);
  channel2 = analogRead(A2);
  channel3 = analogRead(A3);
  channel3 = analogRead(A3);

  channel4 = analogRead(A4);
  channel4 = analogRead(A4);
  channel5 = analogRead(A5);
  channel5 = analogRead(A5);
  channel6 = analogRead(A6);
  channel6 = analogRead(A6);
  channel7 = analogRead(A7);
  channel7 = analogRead(A7);
  
  iteration++;
  //sprintf( buffer, "D %5d   %4d %4d   %4d %4d   %4d %4d   %4d %4d\n", iteration, channel0, channel1, channel2, channel3, channel4, channel5, channel6, channel7 );
  sprintf( buffer, "D %5d \t%d %d \t%d %d \t%d %d \t%d %d\n", 
           iteration, 
           channel0, channel1, channel2, channel3, 
           channel4, channel5, channel6, channel7  );
  Serial.print( buffer );
  
  // delay( 2 );
}
