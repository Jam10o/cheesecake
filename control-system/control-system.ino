
//////////////////////////////////////////////////////////////////////////
/// External libraries
#include "Wire.h"
#include "Servo.h"
#include "math.h"
#include "TinyGPS.h"
#include "Time.h"
#include "software_uart.c"

#define SER //servo (rudder)
#define ROT //rotors
#define GPS //gps
#define COM //compass
#define DEBUG
//#define DATA_PROMPT
#define DATA_PREWRITTEN //only use one of these at a time (not none), either prewrite or prompt for coordinates on startup
#define ROTOR_PIN_1  5
#define ROTOR_PIN_2  6
#define SERVO_PIN 9
// lots of code borrowed from demot
#define CMPS10_ADDRESS         0x60    // I2C address
#define CMPS10_HEADING_REG     0x02    // Bearing
#define rad2deg(x) (180/M_PI) * x
#define deg2rad(x) x * M_PI/180
#define NUM_OF_WAYPOINTS 8
#define WP_THRESHOLD 10
#define DRAG_RACE_HEADING 120
float wp_lats[NUM_OF_WAYPOINTS];
float wp_lons[NUM_OF_WAYPOINTS];
#define GPSPIN 11
TinyGPS gps;
//

Servo rudderServo;
int wp_hdg = 0;
float wp_dist = 0.0;
int wp_num = 0;
float igain = 0.01;
float pgain = 0.1;
float running_err = 0.0;
int hdg_err = 0;
unsigned char rud1pow = 255;
unsigned char rud2pow = 255;
struct Data {
  int heading;
  int rudder;
  float lat;
  float lon;
}
data;

//////////////////////////////////////////////////////////////////////////////////////////
// Utility function that calculates difference between two headings taking wrap around
// into account
int get_hdg_diff(int heading1)
{
  int result;

  result = heading1 - data.heading;

  if (result < -180)
  {
    result = 360 + result;
    return result;
  }

  if (result > 180)
  {
    result = result - 360;
    return result;
  }

}

#ifdef GPS
void readGPS() {
  unsigned long fix_age = 9999;

  //make sure the GPS has a fix, this might cause a wait the first time, but it should
  // be quick any subsequent time
  unsigned long start = millis(); // This times us out, so if the wire comes lose we won't
  // get stuck here forever
  while (millis() < start + 2000)
  {
    // Here we just pass the data over to TinyGPS if we have any
    if (char c = uart_rx(11,4800))
    {
      gps.encode(c);
      // Prints out the characters coming in
      // Each NMEA string ends in a new line character
      if (c == '\n')
      {
        break;
      }
    }


    // Now we ask TinyGPS for the data and store it outselves
    gps.f_get_position(&data.lat, &data.lon, &fix_age);
  }
}

#endif

void headingcalc() {
  wp_hdg = DRAG_RACE_HEADING;
#ifdef GPS
  readGPS();
  wp_hdg = (int) gps.course_to(data.lat, data.lon, wp_lats[wp_num], wp_lons[wp_num]);
  wp_dist = gps.distance_between(data.lat, data.lon, wp_lats[wp_num], wp_lons[wp_num]);
#endif
}

void orientationStuff() {
  //blatant pull of waypoint logic from demot makes me sad... but it looks like it works fine...
  headingcalc();
  // Move onto next waypoint if we are inside the waypoint's radius
  if (wp_dist < WP_THRESHOLD)
  {
    wp_num++;

    if (wp_num == NUM_OF_WAYPOINTS) //reached last waypoint already, keep us here
    {
      wp_num--;
    }
    else //reached new waypoint
    {
      // Work out the new heading and distance
      headingcalc();
    }
  }
}

byte read_i2c(int address, int _register) {
  Wire.beginTransmission(address);
  Wire.write(_register);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  delay(50);
  return Wire.read();
}

int wrap(unsigned int deg){
  deg = deg + 180;
  if(deg > 360){
    deg = deg - 360;
  }
  return deg;
}
  
int readCOM(){
  unsigned int high = read_i2c(CMPS10_ADDRESS, CMPS10_HEADING_REG) << 8;
  unsigned int low = read_i2c(CMPS10_ADDRESS, CMPS10_HEADING_REG+1);
  return wrap((high + low)/10);
}


// merged compass-readings and rudder-turning because it will make later things simpler
#ifdef COM

void turningStuff() {


 
  data.heading = readCOM();
#ifdef SER  
  // apply PI control
  data.rudder = get_hdg_diff(wp_hdg);
  //turn the rudder after the math stuff

  rudderServo.writeMicroseconds(1200 + (data.rudder * 4));
#endif
}
#endif

// end code borrowed from demot

void differentialSteering() { //TODO: this definitely needs testing...
  //slow down rotors if they are being counterproductive, otherwise leave them on
  int rel = get_hdg_diff(wp_hdg);
  if((rel > -15) && (rel < 15)){
    rud1pow = 255;    //if you are within 30 degrees of facing target, full speed.
    rud2pow = 255;
  } 
  else if(rel > 0) { //if you are heading left
    rud1pow = 128;  // slow down rudder 1
  } 
  else if(rel < 0) { //if you are heading right
    rud2pow = 128;  // ^^ the same, rudder 2
  }
  analogWrite(ROTOR_PIN_1, rud1pow);
  analogWrite(ROTOR_PIN_2, rud2pow);
}


//////////////////////////////////////////////////////////////////////////
///
/// Arduino functions
///
//////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  //connect rudder (borrowing alot of code from demot again :P)
  rudderServo.attach(SERVO_PIN,200,2250); // Attach, with the output limited (TODO, look up if these values can be improved)
  rudderServo.write(90); // Centre it roughly
  //makes no difference on 32u4
  unsigned long last_gps_read = 0;
  Wire.begin();
  Serial.println("i2c works");
  pinMode(ROTOR_PIN_1, OUTPUT);
  pinMode(ROTOR_PIN_2, OUTPUT);
  pinMode(GPSPIN, INPUT);

#ifdef DATA_PROMPT
  for (int i = 0; i < NUM_OF_WAYPOINTS; i++) {
    Serial.print("Please enter the latitude of waypoint");
    Serial.println(i + 1);
    while (!Serial.available()) {
      //doesn't continue until serial is available
    }

    wp_lats[i] = Serial.parseFloat();
    Serial.println();
    Serial.print("Please enter the longitude of waypoint");
    Serial.println(i + 1);
    while (!Serial.available()) {
      //doesn't continue until serial is available
      ;
    }

    wp_lons[i] = Serial.parseFloat();
  }
#endif

#ifdef DATA_PREWRITTEN
  wp_lats[0] = 0.0;
  wp_lons[0] = 0.0;

  wp_lats[1] = 0.0;
  wp_lons[1] = 0.0;

  wp_lats[2] = 0.0;
  wp_lons[2] = 0.0;

  wp_lats[3] = 0.0;
  wp_lons[3] = 0.0;

  wp_lats[4] = 0.0;
  wp_lons[4] = 0.0;

  wp_lats[5] = 0.0;
  wp_lons[5] = 0.0;

  wp_lats[6] = 0.0;
  wp_lons[6] = 0.0;

  wp_lats[7] = 0.0;
  wp_lons[7] = 0.0;
#endif


}

void loop()
{
  orientationStuff();
#ifdef COM
  turningStuff();
#endif
#ifdef ROT
  differentialSteering();
#endif
#ifdef DEBUG
  // delay(900);
#else
  //  delay(600);
#endif

#ifdef DEBUG
  Serial.println();
  Serial.print("heading to waypoint: ");
  Serial.println(get_hdg_diff(wp_hdg));
  Serial.println(wp_hdg);
  //delay(300);
  Serial.print("vessel heading: ");
  Serial.println(data.heading);
  // delay(300);
  Serial.print("distance to waypoint: ");
  Serial.println(wp_dist);
  Serial.println(data.rudder);  
#endif

}



