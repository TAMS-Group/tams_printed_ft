/** rectangular_octo_sensor_test using Arduino nano.
 *  
 *  Basic analog readout of 8 (Everlight ITR 8307) optocouplers
 *  using the Arduino nano, wired for the "block-ft" sensor as described
 *  in the IEEE Access paper.
 *  
 *  Uses the internal pullups of the Arduino nano on analog inputs A0..A5,
 *  but requires external pullups (about 50kOhm) on pins A6 and A7.
 *  
 *  Samples the eight analog channels, and prints the values to the Serial
 *  console at 230400 baud (!). Also sends iteration count and an error mask
 *  indicating underload/overload/breakage.
 *  
 *  Sensor ordering is clockwise, starting with top left sensor.
 *  
 *  Initially blinks once to identify the sensor (about 3 secs), then 
 *  enter the endless loop(). Afterwards, only blinks in case of errors,
 *  but the tx LED might indicate device operation (depending on your board).
 */


#define N_CHANNELS 8

int analog_values[N_CHANNELS];
int lower_limits[] = { 50, 50, 50, 50, 50, 50, 50, 50 };
int upper_limits[] = { 950, 950, 950, 950, 950, 950, 950, 950 };

int latest_blink_millis = 0;
int iteration;
int LED_PIN = 13;
int led = 0;

void initialBlinkSequence()
{
  // long-short-long-short-short
  digitalWrite( LED_PIN, HIGH );  delay( 500 ); digitalWrite( LED_PIN, LOW ); delay( 200 ); 
  digitalWrite( LED_PIN, HIGH );  delay( 200 ); digitalWrite( LED_PIN, LOW ); delay( 200 );
  digitalWrite( LED_PIN, HIGH );  delay( 500 ); digitalWrite( LED_PIN, LOW ); delay( 200 ); 
  digitalWrite( LED_PIN, HIGH );  delay( 200 ); digitalWrite( LED_PIN, LOW ); delay( 200 );
  digitalWrite( LED_PIN, HIGH );  delay( 200 ); digitalWrite( LED_PIN, LOW ); delay( 200 );
}



void setup() {
  // put your setup code here, to run once:

  // define analog input pins. Some with internal pullups, others with external pullups.
  // 
  pinMode( A0, INPUT_PULLUP );
  pinMode( A1, INPUT_PULLUP );
  pinMode( A2, INPUT_PULLUP );
  pinMode( A3, INPUT ); // got external 47K
  pinMode( A4, INPUT_PULLUP );
  pinMode( A5, INPUT_PULLUP );
  pinMode( A6, INPUT ); // external 36k pullup!
  pinMode( A7, INPUT ); // external 36k pullup

  // initial blink sequence, long-short-long-short-short to identify sensor
  //
  pinMode( LED_PIN, OUTPUT );
  initialBlinkSequence();

  // initialize serail communication (via usb!) 
  //
  Serial.begin( 230400 ); // ( 115200 );
  Serial.println( "Block-FT sensor (ft-block-octo-8307.ino)..." );
}



void loop() {
  iteration ++;

  // for eight channels, we can only afford 2x oversampling and still get
  // a high sample rate. We could change ADC clock rate etc, but the Arduino
  // nano ADC should be ok with 2x oversampling.
  // 
  for( int i=0; i < N_CHANNELS; i++ ) {
    // analog_values[i] = analogRead( (A0+i) );
    analog_values[i] = analogRead( (A0+i) );
    analog_values[i] = analogRead( (A0+i) );
  }

  // overload? broken? (per channel overload / total sensor destruction)
  bool broken = true;
  bool overload = false;
  int  error_mask = 0; // bit 8: broken, bit 0..7 channel overload/underload
  int  mask = 1;
  for( int c=0; c < N_CHANNELS; c++ ) {
    if (analog_values[c] < lower_limits[c]) error_mask |= mask;
    if (analog_values[c] > upper_limits[c]) error_mask |= mask;
    if (analog_values[c] > lower_limits[c]) broken = false; // at least one channel still has signal
    mask = mask << 1;
  }
  if (error_mask != 0) overload = true;
  if (broken) error_mask |= 0x100;
  

  Serial.print( "D " );
  Serial.print( iteration );
  Serial.print( "\t" );
  Serial.print( error_mask );
  Serial.print( "\t" );
  
  //for( int i=0; i < 8; i++ ) {
  //  Serial.print( analog_values[i] );
  //  Serial.print( "\t" );
  //}
  // sensors are wired in a weird way... sensor is orientated 
  // so that the power-supply cable (red) is at 12-o-clock,
  // with the green cable at 5-o-clock.
  // 
  Serial.print( analog_values[5] ); Serial.print( "\t" ); // left-side-lower (purple),   force0, ADC5
  Serial.print( analog_values[6] ); Serial.print( "\t" ); // left-side-upper (gray),     force1, ADC6

  Serial.print( analog_values[3] ); Serial.print( "\t" ); // right-side-upper (brown),   force2, ADC3
  Serial.print( analog_values[4] ); Serial.print( "\t" ); // right-side-lower (red),     force3, ADC4

  Serial.print( analog_values[0] ); Serial.print( "\t" ); // bottom-side-left (green),   force4, ADC0
  Serial.print( analog_values[7] ); Serial.print( "\t" ); // bottom-side-right (orange), force5, ADC7

  Serial.print( analog_values[1] ); Serial.print( "\t" ); // top-side-left  (purple),    force6, ADC1
  Serial.print( analog_values[2] ); Serial.print( "\t" ); // top-side-right (gray),      force7, ADC2
  
  Serial.println();
  Serial.flush();
  
  // blinking indicates error
  int now = millis();

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
    if ((now - latest_blink_millis) > 1000) { // reset overload flag after 1 second
      latest_blink_millis = 0; 
    }  
  }

  delay( 1 );
}
