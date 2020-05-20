/* screwdriver_v3_8x.ino
 *  
 * Example firmware for the "Instrumented Screwdriver" sensing object
 * with the Arduino Nano 33 IOT board. Measures gyro+acc, 5x tool forces, 
 * 12x grasp forces from on-board IMU and multiplexed optical proximity 
 * sensors. Data is written to Serial (if connected) and sent via WiFi UDP.
 * 
 * Using fast analogRead() with 8x oversampling and median-filtering
 * to reduce the Wifi-induced noise.
 * 
 * 2020.05.13 - updated (add 8x oversampling and median filtering)
 * 2020.04.15 - updated (v2)
 * 2019.08.21 - created
 * 
 * (C) 2019, 2020,   fnh, hendrich@informatik.uni-hamburg.de
 */


#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>   // IMU
#include "arduino_secrets.h"   // Wifi password. Provide your own values here.


// main configuration options: do we want serial communication?
// do we want WiFI communication?
// 
unsigned int try_serial = 1;
unsigned int have_serial = 0; 
unsigned int use_wifi = 1;
unsigned int use_wifi_input = 0;

#define debug 0
#define LED_PIN 13


// maximum number of tries/wait seconds to open Serial port
#define MAX_SERIAL_ATTEMPTS 10     

// WIFI stuff 
int status = WL_IDLE_STATUS;

char ssid[] = SECRET_SSID;     // your network SSID (name)
char pass[] = SECRET_PASS;     // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;              // your network key Index number (needed only for WEP)


unsigned int localPort = 11333; // local port to listen for UDP packets
IPAddress tamsXXX( 192, 168, 0, 1 );
WiFiUDP   Udp;   // UDP instance to send and receive packets over UDP



#define PACKET_SIZE   255
char packetBuffer[ PACKET_SIZE]; //buffer to hold incoming and outgoing packets
char* pp = packetBuffer;   // packet-pointer: active char in buffer




const int adc_warmup = 1; // was: 3 // extra initial ADC reads before actual read

// analog channels for screwdriver tool forces
int force0, force1, force2, force3, force4, force5;

// analog channels for screwdriver grasping handle
int touch_0_lower, touch_0_upper;
int touch_1_lower, touch_1_upper;
int touch_2_lower, touch_2_upper;
int touch_3_lower, touch_3_upper;
int touch_4_lower, touch_4_upper;
int touch_5_lower, touch_5_upper;


unsigned int iteration = 0;


// if needed, scale IMU values back from float to (signed int 16),
// current library sets 2000 deg/sec gyro range and 4G acc range, 104 Hz,
// see ~/Arduino/Libaries/Arduino_Arduino_LSM6DS3/src for sources...
// 
#define GYRO_TO_INT   (32768.0 / 2000.0)
#define ACC_TO_INT    (32768.0 / 4.0)


void msg( String s ) {
  if (have_serial) Serial.println( s );
}


/**
 * generate a diagnostic blink pattern on build-in LED (pin 13),
 * with the given number of long (500 msec) blinks, followed by
 * short (200 mesc) blinks, repeated n times.
 */
void info_blink_led( int n_long, int n_short, int b_repetitions ) {
  for( int i=0; i < max( 1, b_repetitions); i++ ) {
    for( int j=0; j < n_long; j++ ) {
      digitalWrite( LED_PIN, HIGH );
      delay( 500 );
      digitalWrite( LED_PIN, LOW );
      delay( 200 );
    }
    for( int j=0; j < n_short; j++ ) {
      digitalWrite( LED_PIN, HIGH );
      delay( 200 );
      digitalWrite( LED_PIN, LOW );
      delay( 200 );
    }
  } // for i
}


/** 
 * Outputs a diagnostic blink pattern on built-in LED (pin 13):
 * count short (0.2 sec) blinks, followed by 1 sec dark.
 * Note: this function does NOT return.
 */
void error_blink_forever( int count ) {
  pinMode( 13, OUTPUT );
  
  while( 1 ) {
    for( int i=0; i < count; i++ ) {
      digitalWrite( 13, HIGH );
      delay( 200 );
      digitalWrite( 13, LOW );
      delay( 200 );
    }
    delay( 1000 );
  }
}


// Array A[] has the items to sort; array B[] is a work array.
void TopDownMergeSort(int A[], int B[], int n)
{
    CopyArray(A, 0, n, B);           // one time copy of A[] to B[]
    TopDownSplitMerge(B, 0, n, A);   // sort data from B[] into A[]
}


// Sort the given run of array A[] using array B[] as a source.
// iBegin is inclusive; iEnd is exclusive (A[iEnd] is not in the set).
void TopDownSplitMerge( int B[], int iBegin, int iEnd, int A[])
{
    if(iEnd - iBegin < 2)                       // if run size == 1
        return;                                 //   consider it sorted
    // split the run longer than 1 item into halves
    int iMiddle = (iEnd + iBegin) / 2;              // iMiddle = mid point
    // recursively sort both runs from array A[] into B[]
    TopDownSplitMerge(A, iBegin,  iMiddle, B);  // sort the left  run
    TopDownSplitMerge(A, iMiddle,    iEnd, B);  // sort the right run
    // merge the resulting runs from array B[] into A[]
    TopDownMerge(B, iBegin, iMiddle, iEnd, A);
}

//  Left source half is A[ iBegin:iMiddle-1].
// Right source half is A[iMiddle:iEnd-1   ].
// Result is            B[ iBegin:iEnd-1   ].
void TopDownMerge(int A[], int iBegin, int iMiddle, int iEnd, int B[])
{
    int i = iBegin; int j = iMiddle;
 
    // While there are elements in the left or right runs...
    for (int k = iBegin; k < iEnd; k++) {
        // If left run head exists and is <= existing right run head.
        if (i < iMiddle && (j >= iEnd || A[i] <= A[j])) {
            B[k] = A[i];
            i = i + 1;
        } else {
            B[k] = A[j];
            j = j + 1;
        }
    }
}


void CopyArray(int A[], int iBegin, int iEnd, int B[])
{
    for(int k = iBegin; k < iEnd; k++)
        B[k] = A[k];
}



int median_sort_buffer[32];


int analog_8x_median( int channel ) {
  int raw[8];
  int dummy = analogRead( channel ); // cold-start, throw away
  
  raw[0] = analogRead( channel );
  raw[1] = analogRead( channel );
  raw[2] = analogRead( channel );
  raw[3] = analogRead( channel );
  raw[4] = analogRead( channel );
  raw[5] = analogRead( channel );
  raw[6] = analogRead( channel );
  raw[7] = analogRead( channel );

  // sort raw -> back into raw
  TopDownMergeSort( raw, median_sort_buffer, 8 );

  /* debugging only
  for( int i=0; i < 8; i++ ) {
    Serial.println( raw[i] ); Serial.print( "\t" );
  }
  Serial.println();
  */

  // int value = (raw[2]+raw[3]+raw[4]+raw[5]) >> 2;
  // int value = (raw[3]+raw[4]) >> 1; // too noisy
  int value = (raw[0]+raw[1]+raw[2]+raw[3]) >> 2;
  // int value = (raw[0]+raw[1]) >> 1; // ok on usb, very very spiky on lipo 3.7v + step-up
  
  return value;
}



void setup() {
  // initial blinking, informing the user in case of no Serial connected...
  // 
  pinMode( LED_PIN, OUTPUT );
  
  // try to open serial port, remember in have_serial
  //
  if (try_serial) {
    for( int n=0; n < MAX_SERIAL_ATTEMPTS; n++ ) {
      // open serial communications and wait for port to open:
      Serial.begin( 500000 ); // 115200 9600
      delay( 1000 );
 
      if (Serial) { 
        have_serial = 1; 
        msg( "Instrumented Screwdriver v3..." );
        break; 
      }
    }
    info_blink_led( 1, 1, 1 ); // 1x long short short short short short
  }

  // digital and analog I/O pins setup
  //
  analogReadResolution( 10 );

  // Initialize Analog Controller
  // Setting clock
  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |    // was: 512 Divide Clock by 512, then 128 (v2), now 64.
                   ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default

  ADC->SAMPCTRL.reg = 0x0f; // was: 0x3f;                        // Set max Sampling Time Length

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0

  analogReference( AR_DEFAULT ) ; // Analog Reference is AREF pin (3.3v)


  // optocoupler setup for the screwdriver "tool plate". 
  // D10-D11-D12 drive chains of two emitter LEDs each,
  // while A0 and A1 are connected to the upper/lower phototransistors of
  // each chain...
  pinMode( 10, OUTPUT ); // dark red, left-right LEDs
  pinMode( 11, OUTPUT ); // gray, center LED
  pinMode( 12, OUTPUT ); // bright red, top-bottom LEDs

  pinModeHighDriveStrength( 10, true );
  pinModeHighDriveStrength( 11, true );
  pinModeHighDriveStrength( 12, true );
  
  pinMode( A0, INPUT ); // top-left (front-view of the LEDs)
  pinMode( A1, INPUT ); // bottom-right (front-view of the LEDs)
  pinMode( A2, INPUT ); // center LED

  // PULLUPs dont work? try digitalWrite...
  digitalWrite( A0, HIGH );
  digitalWrite( A1, HIGH );
  digitalWrite( A2, HIGH );

  // optocoupler setup for the graping handle with six chains
  // of two optocouplers, driven by D3-D4-D5-D6-D7-D8. 
  // Phototransistors are connected to A6 and A7.
  pinMode( 3, OUTPUT );
  pinMode( 4, OUTPUT );
  pinMode( 5, OUTPUT );
  pinMode( 6, OUTPUT );
  pinMode( 7, OUTPUT );
  pinMode( 8, OUTPUT );

  pinModeHighDriveStrength( 3, true );  
  pinModeHighDriveStrength( 4, true );  
  pinModeHighDriveStrength( 5, true );  
  pinModeHighDriveStrength( 6, true );  
  pinModeHighDriveStrength( 7, true );  
  pinModeHighDriveStrength( 8, true );  
  
  // pinMode( A3, INPUT_PULLUP );
  // note: A4=SDA 
  // note: A5=SCL
  pinMode( A6, INPUT );
  pinMode( A7, INPUT );

  info_blink_led( 1, 2, 1 ); // I/O setup ok

  // initialize IMU, enter error loop if this fails.
  //
  if (!IMU.begin()) {
    msg( "Failed to initialize the IMU! Stopped." );
    error_blink_forever( 3 );
  }

  info_blink_led( 1, 3, 1 ); // IMU ok

  // check for the WiFi module:
  //
  int attempts = 0;
  if (use_wifi) {
    if (WiFi.status() == WL_NO_MODULE) {
      msg( "Communication with WiFi module failed! Stopped." );
      error_blink_forever( 4 );
    }

    String fv = WiFi.firmwareVersion();
    if (fv < "1.0.0") {  
      msg( "Please upgrade the WifiNINA firmware" );
    }

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
      if (have_serial) {
        Serial.print( "Attempting to connect to SSID: " );
        Serial.println(ssid);
      }
    
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin( ssid, pass );

      info_blink_led( 2, 2, 1 );
      
      status = WiFi.status();
      for( int j=0; j < 30; j++ ) {
        if (status == WL_CONNECTED) break;
        status = WiFi.status();
        if (have_serial) { Serial.print( "Wifi status is " ); Serial.print( status ); Serial.print( " attempts: " ); Serial.println( attempts ); }
        delay( 500 );
      }
    }
  }

  // disable low-power modes
  if (use_wifi) {
    msg( "Connected to WiFi!" );
    if (have_serial) printWifiStatus();

     WiFi.noLowPowerMode();
     Udp.begin( localPort );
  }

  info_blink_led( 1, 4, 1 );

  iteration = 0;
  
} // end setup


long lastLedSwitchMicros = 0;
long lastLedSwitchState = 0;

float ax, ay, az;
float gx, gy, gz;
int iax, iay, iaz, igx, igy, igz;




void loop() {
  iteration ++;
  pp = packetBuffer;

  long t0 = micros();
  if ((t0 - lastLedSwitchMicros) > 250000L) {
    lastLedSwitchMicros = t0;
    lastLedSwitchState ^= 1;
    digitalWrite( LED_PIN, lastLedSwitchState );
  }

  // pins D12-D13-D14 drive the LEDs on the tool-plate,
  // analog inputs are A0 and A1. Read each analog channel
  // a few times to ensure stable values behind the ADC
  // analog multiplexer.
  //
  // 2020.01.25: we need extra pause when switching muxes, 
  // probably due the response time of the phototransistor.
  // At the moment, sample A0,A2, then sample A1, then sample A0,A2 again.
  // This needs to be considered also for the grasp handle sensors...
  // 
  int t1 = micros();

  int nt = sprintf( pp, "D %d T", iteration );
  pp += nt;
  
  digitalWrite( 10, HIGH );
  digitalWrite( 11, LOW );
  digitalWrite( 12, LOW );
  force0 = sampleADCIntoPacket( A0, 3 ); // adc_warmup );
  force1 = sampleADCIntoPacket( A2, 1 ); // adc_warmup );

  digitalWrite( 10, LOW );
  digitalWrite( 11, LOW );
  digitalWrite( 12, HIGH ); 
  force2 = sampleADCIntoPacket( A0, 5 ); // adc_warmup );
  force3 = sampleADCIntoPacket( A2, 1 ); // adc_warmup );

  digitalWrite( 10, LOW );
  digitalWrite( 11, HIGH );
  digitalWrite( 12, LOW );
  force4 = sampleADCIntoPacket( A1, 3 ); // adc_warmup );
  force5 = 0;


  digitalWrite( 10, LOW );
  digitalWrite( 11, LOW );
  digitalWrite( 12, LOW );


  // read grasp-handle forces from channels A6+A7
  // 
  int t2 = micros();
  int nh = sprintf( pp, " H" );
  pp += nh;
  
  one_hot( 0x04 ); 
  touch_0_lower = sampleADCIntoPacket( A6, 2 ); // master (blue ribbon cable, near USB cable pos)
  touch_0_upper = sampleADCIntoPacket( A7, 2 );

  one_hot( 0x08 );
  touch_1_lower = sampleADCIntoPacket( A6, 2 ); // purple
  touch_1_upper = sampleADCIntoPacket( A7, 2 );

  one_hot( 0x10 );
  touch_2_lower = sampleADCIntoPacket( A6, 2 ); // gray, above antenna
  touch_2_upper = sampleADCIntoPacket( A7, 2 );

  one_hot( 0x20 );
  touch_3_lower = sampleADCIntoPacket( A6, 2 ); // green 
  touch_3_upper = sampleADCIntoPacket( A7, 2 );

  one_hot( 0x40 );
  touch_4_lower = sampleADCIntoPacket( A6, 2 ); // orange
  touch_4_upper = sampleADCIntoPacket( A7, 2 ); 

  one_hot( 0x80 );
  touch_5_lower = sampleADCIntoPacket( A6, 2 ); // red
  touch_5_upper = sampleADCIntoPacket( A7, 2 );

  one_hot( 0x00 );

  // read IMU values (if available)
  // 

  int t3 = micros();
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope( gx, gy, gz );
    igx = (int) (gx * GYRO_TO_INT);
    igy = (int) (gy * GYRO_TO_INT);
    igz = (int) (gz * GYRO_TO_INT);
  }
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration( ax, ay, az );
    iax = (int) (ax * ACC_TO_INT);
    iay = (int) (ay * ACC_TO_INT);
    iaz = (int) (az * ACC_TO_INT);
  }

  int ng = sprintf( pp, " G %d %d %d", igx, igy, igz );
  pp += ng;
  
  int na = sprintf( pp, " A %d %d %d", iax, iay, iaz );
  pp += na;
                        
  *pp= '\n'; pp++;
  *pp = 0; pp++;
  

  int t4 = micros();

  if (use_wifi) {
    Udp.beginPacket( tamsXXX, 11333 ); 
    int nb = (pp - packetBuffer);
    Udp.write(packetBuffer, nb );
    Udp.endPacket();
  }

  int t5 = micros();

  if (have_serial) {
    Serial.print( "loop took " ); Serial.print( (t5-t1) ); Serial.print( " usec." );
    Serial.print( "forces,grasps,imu,udp took " );
    Serial.print( (t2-t1) ); Serial.print( " " );
    Serial.print( (t3-t2) ); Serial.print( " " );
    Serial.print( (t4-t3) ); Serial.print( " " );
    Serial.print( (t5-t4) ); Serial.println( " " );
  }
  
  if (have_serial) {
    Serial.print( packetBuffer );           
    // Serial.flush();
  }
  
  // wait to see if a reply is available
  // delay(10); // results in ~60Hz without ADC reads
  // delay( 1 );

  if (use_wifi && use_wifi_input) {
    if (Udp.parsePacket()) {
      if (have_serial) Serial.println("packet received: `");
        // We've received a packet, read the data from it
      Udp.read(packetBuffer, PACKET_SIZE); // read the packet into the buffer
      if (have_serial) Serial.print( packetBuffer ); // echo incoming command to serial 
    }
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


void one_hot( unsigned int mask ) {

  digitalWrite( 3, (mask&0x04) > 0 ? HIGH : LOW );
  digitalWrite( 4, (mask&0x08) > 0 ? HIGH : LOW );
  digitalWrite( 5, (mask&0x10) > 0 ? HIGH : LOW );
  digitalWrite( 6, (mask&0x20) > 0 ? HIGH : LOW );
  digitalWrite( 7, (mask&0x40) > 0 ? HIGH : LOW );
  digitalWrite( 8, (mask&0x80) > 0 ? HIGH : LOW );
  /* */
}


int sampleADC_spiky( int pin, int start_adc_warmup ) {
  int value;
  for( int i=0; i < start_adc_warmup; i++ ) {
    value = analogRead( pin );
  }
  return value;  
}


static int dummy; // don't let the compiler optimize this away...

/**
 * sample the given analog input pin, using the given number of ADC warmup cycles.
 * Afterwards, we sample the pin a few times and return the average value.
 */
int sampleADC( int pin, int start_adc_warmup ) {

  // start_adc_warmup = 0;
  for( int i=0; i < start_adc_warmup; i++ ) {
    dummy += analogRead( pin );
  }

  /*
  int v1 = analogRead( pin );
  int v2 = analogRead( pin );
  return (v1+v2)/2;
  */ 
  // return analogRead( pin );
  
  int v1 = analogRead( pin );
  int v2 = analogRead( pin );
  int v3 = analogRead( pin );
  
  return min( v3, min( v1, v2 )); // Wifi spikes mean higher current, lower supply voltage, less light, lower phototransistor current, higher ADC readout
}


/**
 * sample given ADC channel and write value into packetBuffer at current
 * pp pointer position. Finally, return sampled analog value,
 */
int sampleADCIntoPacket( int pin, int start_adc_warmup ) {
  // int value = sampleADC( pin, start_adc_warmup ); // v2
  int value = analog_8x_median( pin ); // v3
  int nchars = sprintf( pp, " %d", value );
  pp += nchars;

  if (have_serial && debug) {
    Serial.print( "sampleADCIntoPacket: pin " ); Serial.print( pin );
    Serial.print( " value " ); Serial.print( value );
    Serial.print( " nchars " ); Serial.print( nchars );
    Serial.print( " pp " ); Serial.print( (pp - packetBuffer) );
    Serial.println();
  }
  
  return value;
}



/**
 * select high (True) or low (false) drive strength on given digital I/O pin.
 * See SAMD docs and "variants.h" for details.
 */
void pinModeHighDriveStrength( uint32_t ulPin, boolean b ) {
  uint8_t val = (uint8_t) (PORT_PINCFG_INEN);
  if (b == true) val |= (uint8_t) (PORT_PINCFG_DRVSTR);
  
  PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg = val;
  //    = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR) ;
  PORT->Group[g_APinDescription[ulPin].ulPort].DIRSET.reg = (uint32_t)(1<<g_APinDescription[ulPin].ulPin) ;
}
