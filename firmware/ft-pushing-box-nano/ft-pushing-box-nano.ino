/* ft-pushing-box-nano.ino

  2020.02.16 - created
  
  (c) 2020, hendrich@informatik.uni-hamburg.de  
 */
 
#include <stdio.h>

// define eight channels for compatibility with the bigger sensors, but
// only two are actually used on the pushing box.
// 
#define N_CHANNELS 8                
#define USE_INTERNAL_PULLUPS (true)

 
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
  
  // do we want internal pullups? 
  // note: Arduino nano has no internal pullups on A6, A7:
  // make sure to connect external pullups here!
  // 
  {  
    pinMode( A0, INPUT_PULLUP );
    pinMode( A1, INPUT_PULLUP );
  }

  // we use digital pins to drive the pullup resistors, with
  // the resistors soldered across the board to the matching
  // analog pin...
  //
  /*
  for( int di=2; di <= 9; di++  ) {
    pinMode( di, OUTPUT );    
    digitalWrite( di, HIGH );
  }
  */
  
  // use the LED to indicate board status via blinking
  pinMode( LED_PIN, OUTPUT);

  // initialize ADC default values for lower/upper limits
  for( int c=0; c < N_CHANNELS; c++ ) {
    lower_limits[c] =   50;
    upper_limits[c] = 1000;
    channels[c] = 0;
  }

  // initialize serial communication; wait until ready. Also generates
  // a blink pattern to identify this device (short-short-short-long-pause).
  //
  Serial.begin(115200);
  int blink_iterations = 0;

  
  while( !Serial || (blink_iterations < 1)) { // short-short-short-long-pause  (Teensy vs. Modemmanager?)
    for( int i=0; i < 3; i++ ) {
      digitalWrite( LED_PIN, 1 );
      delay( 150 );
      digitalWrite( LED_PIN, 0 );
      delay( 150 );
    }
    digitalWrite( LED_PIN, 1 );
    delay( 300 );
    digitalWrite( LED_PIN, 0 );
    delay( 1000 );
    blink_iterations++;
  }

}


// the loop routine runs over and over again forever:
void loop() {

  // read the analog inputs: for now, 4x oversampling 
  //
  channels[0] =  analogRead( A0 );
  channels[0] += analogRead( A0 );
  channels[0] += analogRead( A0 );
  channels[0] += analogRead( A0 );
  channels[0] >>= 2;

  
  channels[1] =  analogRead( A1 );
  channels[1] += analogRead( A1 );
  channels[1] += analogRead( A1 );
  channels[1] += analogRead( A1 );
  channels[1] >>= 2;

  int error_mask = 0;

  // print out the value you read: format is common to the other sensors,
  // with iteration count, error_mask, and eight channel values - but only
  // channels 0 and 1 are actually measured by this device.
  // 
  iteration++;
  sprintf( buffer, "D %2d %5d  %5d %5d   %5d %5d   %5d %5d  %5d %5d\n", 
           iteration, error_mask,
           channels[0], channels[1], channels[2], channels[3],
           channels[4], channels[5], channels[6], channels[7] );
  Serial.print( buffer );
  Serial.flush(); // wait until Serial port idle before sampling ADC again

  // sleep a bit to define the effective sample rate
  // 
  delay( 5 );
}
