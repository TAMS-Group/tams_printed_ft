/* ft-big-octo-1103.ino

  Optical force-torque sensor using eight Vishay TCST1103 optocoupler 
  (organized as four "2DOF-sensor modules" with a pair of sensors each) 
  connected to an Arduino nano.

  Note: on some Arduino clones, also try the "old bootloader" 
  if the IDE reports programming failure.
  
  Connect the analog inputs to A0..A7, and values are then streamed 
  via serial console at 115200 baud.
  
  The on-board LED (pin 13) displays sensor status like this:

  - Serial not connected: blinking short-short-short-long-pause
  - device working: continuous light
  - single-channel underload/overload: 2Hz blinking
  - all-channels underload (spring broken?): SOS blinking

  Note that we currently use the Arduino nano 3.3V supply to
  drive the LEDs; make sure to replace the LED series resistor
  when using 5V supply voltage.

  Cable encoding for the "2DOF sensor modules" is:

  black  - GND
  red    - 3V3 LED power (2LEDs + 100Ohm resistor, roughly 10mA per pair)
           5V LED power (roughly 20mA per LED pair)
  purple - phototransistor 1 (D connected to GND, so needs pullup)
  gray   - phototransistor 2 (D connected to GND, so needs pullup)

  For best voltage swing using the Arduino nano and the "taped" 1103's,
  we use external 100 kOhm pullup resistors. To avoid an extra circuit
  board, the pullups are placed across the board and driven by digital
  pins D2..D9, together with digitalWrite( D2..D9, HIGH ). Of course,
  5V could also have been used for the pullups, but connecting them
  to 3V3 would lose about 25% of the ADC range.

  2020.02.09 - decide on pullup resistor strategy 
  2020.02.08 - created (based on ft-everlight-hex-9904)
  
  (c) 2018, 2019, 2020, hendrich@informatik.uni-hamburg.de  
 */
 
#include <stdio.h>

#define N_CHANNELS 8
#define USE_INTERNAL_PULLUPS (false)
#define ADC_IDLE_READS   (2)
#define ADC_READ_REPETITIONS  (4)

 
int channels[ N_CHANNELS ];
int lower_limits[ N_CHANNELS ];
int upper_limits[ N_CHANNELS ];

int iteration;
int led = 0;
int LED_PIN = 13;

int latest_blink_millis = 0;
int t_now;
 
 
char buffer[120];


 

// the setup routine runs once when you press reset:
void setup() {
  // analogReadResolution( 13 ); // only for Teensy 3.2+
  
  // do we want internal pullups? For now, all external.
  // note: Arduino nano has no internal pullups on A6, A7:
  // make sure to connect external pullups here!
  // 
  {  
    pinMode( A0, INPUT );
    pinMode( A1, INPUT );
    pinMode( A2, INPUT );
    pinMode( A3, INPUT );
    pinMode( A4, INPUT );
    pinMode( A5, INPUT );
  }
  pinMode( A6, INPUT );
  pinMode( A7, INPUT );

  // we use digital pins to drive the pullup resistors, with
  // the resistors soldered across the board to the matching
  // analog pin...
  //
  for( int di=2; di <= 9; di++  ) {
    pinMode( di, OUTPUT );    
    digitalWrite( di, HIGH );
  }
  
  // use the LED to indicate board status via blinking
  pinMode( LED_PIN, OUTPUT);

  // initialize ADC default values for lower/upper limits
  for( int c=0; c < N_CHANNELS; c++ ) {
    lower_limits[c] =   50;
    upper_limits[c] = 1000;
  }

  // initialize serial communication; wait until ready.
  // At least one full blink iterations, so that we can identify
  // the sketch by watching the blinking pattern.
  //
  Serial.begin(115200);
  int blink_iterations = 0;
  while( !Serial || (blink_iterations < 1)) { // short-short-long-long-pause at least once
    for( int i=0; i < 2; i++ ) {
      digitalWrite( LED_PIN, 1 );
      delay( 150 );
      digitalWrite( LED_PIN, 0 );
      delay( 150 );
    }
    for( int i=0; i < 2; i++ ) {
      digitalWrite( LED_PIN, 1 );
      delay( 300 );
      digitalWrite( LED_PIN, 0 );
      delay( 300 );
    }
    digitalWrite( LED_PIN, 0 );
    delay( 1000 );
    blink_iterations++;
  }
}


// the loop routine runs over and over again forever:
void loop() {

  // read the analog inputs: warmup...
  //
  for( int c=0; c < N_CHANNELS; c++ ) {
    for( int i=0; i < ADC_IDLE_READS; i++ ) {
      analogRead( A0+c );
    }

    // sampling
    channels[c] = 0;
    for( int i=0; i < ADC_READ_REPETITIONS; i++ ) {
      channels[c] += analogRead( A0+c );
    }
    channels[c] /= ADC_READ_REPETITIONS;
  }

  // overload? broken?
  bool broken = true;
  bool overload = false;
  int  error_mask = 0; // bit 8: broken, bit 0..7 channel overload/underload
  int  mask = 1;
  for( int c=0; c < N_CHANNELS; c++ ) {
    if (channels[c] < lower_limits[c]) error_mask |= mask;
    if (channels[c] > upper_limits[c]) error_mask |= mask;
    if (channels[c] > lower_limits[c]) broken = false,
    mask = mask << 1;
  }
  if (error_mask != 0) overload = true;
  if (broken) error_mask |= 0x100;

  // print out the value you read:
  iteration++;
  sprintf( buffer, "D %2d %d  %d %d %d %d   %d %d %d %d\n", 
           iteration, error_mask,
           channels[0], channels[1], channels[2], channels[3],
           channels[4], channels[5], channels[6], channels[7] );
  Serial.print( buffer );

  // blinking indicates error
  int now = millis();
  if (overload) latest_blink_millis = now; // remember current overload
  else if ((now - latest_blink_millis) > 1000) { // reset overload flag after 1 second
    latest_blink_millis = 0;
  }

  if (broken) {
    if ((now - latest_blink_millis) > 100) {
      led = led ^ 0x1;
      digitalWrite( LED_PIN, led ); 
      latest_blink_millis = now;
    }
  }
  else if (overload) {
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

  Serial.flush(); // wait until Serial port idle before sampling ADC again
  // delay( 1 );
}
