

//////////////////////////////////////////////////////////////////////////
/// External libraries
#include "Wire.h"
#include "Servo.h"
#include "math.h"
#include "TinyGPS.h"
#include "Time.h"
#include "SoftwareSerial.h"
#include "EEPROM.h"

#define DEBUG
#define DATA_PROMPT
//#define DATA_PREWRITTEN //only use one of these at a time (not none), either prewrite or prompt for coordinates on startup
#define ROTOR_PIN_1  5
#define ROTOR_PIN_2  6
// lots of code borrowed from demot
#define CMPS10_ADDRESS         0x1E    // I2C address
#define CMPS10_HEADING_REG     0x01    // Bearing, pitch and roll register
#define rad2deg(x) (180/M_PI) * x
#define deg2rad(x) x * M_PI/180
#define NUM_OF_WAYPOINTS 8
#define WP_THRESHOLD 10
float wp_lats[NUM_OF_WAYPOINTS]; 
float wp_lons[NUM_OF_WAYPOINTS];

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

  wp_lats[7]= 0.0;
  wp_lons[7]= 0.0;
#endif


TinyGPS gps;
//

Servo rudderServo;
int wp_hdg=0;
float wp_dist=0.0;
int wp_num=0;
float igain=0.01;
float pgain=0.1;
float running_err=0.0;
int hdg_err=0;
SoftwareSerial gps_serial(11, 12);  // Creates a serial object which allows us to read serial data from pin 11 and write serial data 
// using pin 12.
struct Data{
  uint16_t heading;
  int8_t rudder;
  float lat;
  float lon;
  long unix_time;
} 
data;


//////////////////////////////////////////////////////////////////////////////////////////
// Utility function that keeps angles betweeen 0 and 360
int mod(int value){
  int newValue;
  if(value < 0){
    newValue = value + 360;
  }
  else if(value >= 360){
    newValue = value - 360;
  }
  else{
    newValue = value;
  }
  return newValue;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Utility function that calculates difference between two headings taking wrap around 
// into account
int get_hdg_diff(int heading1,int heading2)
{
  int result;

  result = heading1-heading2;

  if(result<-180)
  {
    result = 360 + result;
    return result;
  } 

  if(result>180)
  {
    result = 0 - (360-result);
  }

  return result;
}


// got rid of a bunch of debug stuff so this is likely to be buggy and broken :)
void readGPS() {
  unsigned long fix_age=9999,time,date;

  //make sure the GPS has a fix, this might cause a wait the first time, but it should 
  // be quick any subsequent time
  while(fix_age == TinyGPS::GPS_INVALID_AGE||fix_age>3000)
  {
    unsigned long start = millis(); // This times us out, so if the wire comes lose we won't 
    // get stuck here forever
    while(millis()<start+2000)
    {
      // Here we just pass the data over to TinyGPS if we have any
      if(gps_serial.available())
      {
        int c = gps_serial.read();
        gps.encode(c);
        // Prints out the characters coming in
        Serial.write(c);
        // Each NMEA string ends in a new line character
        if(c=='\n')
        {
          break;
        }
      }
    }

    // Now we ask TinyGPS for the data and store it outselves
    gps.get_datetime(&date,&time,&fix_age);
  }

  int year;
  byte month,day,hour,min,sec;
  unsigned long age;
  gps.crack_datetime(&year,&month,&day,&hour,&min,&sec,NULL,&age);  
  setTime(hour,min,sec,day,month,year);


}

void orientationStuff() {
  //blatant pull of waypoint logic from demot makes me sad... but it looks like it works fine...
  wp_hdg = (int) gps.course_to(data.lat, data.lon, wp_lats[wp_num],wp_lons[wp_num]);
  wp_dist = gps.distance_between(data.lat, data.lon, wp_lats[wp_num],wp_lons[wp_num]);

  // Move onto next waypoint if we are inside the waypoint's radius
  if(wp_dist<WP_THRESHOLD)
  {       
    wp_num++;

    if(wp_num==NUM_OF_WAYPOINTS) //reached last waypoint already, keep us here
    {
      wp_num--;          
    }
    else //reached new waypoint
    {
      // Work out the new heading and distance
      wp_hdg = (int) gps.course_to(data.lat, data.lon,wp_lats[wp_num],wp_lons[wp_num]);
      wp_dist = gps.distance_between(data.lat, data.lon, wp_lats[wp_num],wp_lons[wp_num]);
    }
  }
}

// merged compass-readings and rudder-turning because it will make later things simpler

int turningStuff() {
  byte msb, lsb;

  Wire.beginTransmission(CMPS10_ADDRESS); // Start communicating with the CMPS10 compasss
  Wire.write(CMPS10_HEADING_REG); // Send the address of the register that we want to read
  Wire.endTransmission();
  Wire.requestFrom(CMPS10_ADDRESS, 2); // Request six bytes of data from the CMPS10 compass, 
  // if you look at the data sheet you will see that the 
  // data we want is stored in a 6 byte register
  
  
  while(Wire.available() < 2); // Busy wait while there is no byte to receive
  msb = Wire.read();
  lsb = Wire.read();
	
	// get a heading in radians
  int heading = (msb << 8) + lsb;

	// return the heading in degrees, 57.29582 = 180 / PI
  return heading / 10;

  hdg_err = get_hdg_diff(wp_hdg,data.heading);

  running_err = running_err + (float)hdg_err;

  // clip the running error to -4000..4000
  if(running_err>4000)
    running_err=4000;
  else if(running_err<-4000)
    running_err=-4000;

  // apply a geometric decay to the running error
  running_err = running_err * 0.9;   

  // apply PI control
  data.rudder = (int) round((pgain * (float)hdg_err) + (igain * running_err));
  if(data.rudder<-5)
  {
    data.rudder=-5;
  }
  else if(data.rudder>5)
  {
    data.rudder=5;
  }
  //turn the rudder after the math stuff
  rudderServo.writeMicroseconds(1500+(data.rudder*100));
};


// end code borrowed from demot

int relhed() {
int relhed = get_hdg_diff(wp_hdg,data.heading);
return relhed;
}

void differentialSteering() { //TODO: this definitely needs testing...
//slow down rotors if they are being counterproductive, otherwise leave them on
int rel = relhed();
unsigned char rudpow = 255;
if ((rel > 25) and (rel < 155)){
rudpow = rudpow / 2;
analogWrite(ROTOR_PIN_1,rudpow); //low
}
else{
rudpow = 255;
analogWrite(ROTOR_PIN_1,rudpow); //high
};
if ((rel < 335) and (rel > 155)){
rudpow = rudpow / 2;
analogWrite(ROTOR_PIN_2,rudpow); //low
}
else{
rudpow = 255;
analogWrite(ROTOR_PIN_2,rudpow); //high
};

};

void saveStatus() {

}

//////////////////////////////////////////////////////////////////////////
///
/// Helper functions
/// 
//////////////////////////////////////////////////////////////////////////

/**
 * A helper function for setting a register in a I2C device.
 */
void i2c_write(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

//////////////////////////////////////////////////////////////////////////
///
/// Arduino functions
/// 
//////////////////////////////////////////////////////////////////////////

void setup()
{
  //connect rudder (borrowing alot of code from demot again :P)
  rudderServo.attach(5, 1060, 1920); // Attach, with the output limited (TODO, look up if these values can be improved)
  rudderServo.writeMicroseconds(1500); // Centre it roughly
  Serial.begin(9600); //makes no difference on 32u4
  gps_serial.begin(4800); // setups serial communications for the GPS
  unsigned long last_gps_read=0;
  Wire.begin();
  pinMode(ROTOR_PIN_1,OUTPUT);
  pinMode(ROTOR_PIN_2,OUTPUT);

#ifdef DATA_PROMPT
  for (int i = 0; i < NUM_OF_WAYPOINTS; i++){
    Serial.print("Please enter the latitude of waypoint");
    Serial.println(i+1);
  while (!Serial.available()) {
   //doesn't continue until serial is available
  }
  
  wp_lats[i] = Serial.parseFloat();
  Serial.println();
  Serial.print("Please enter the longitude of waypoint");
  Serial.println(i+1);
	while (!Serial.available()) {
   //doesn't continue until serial is available
  ;
  }
  
 wp_lons[i] = Serial.parseFloat();
 }
#endif


}

void loop()
{
  orientationStuff();
  data.heading = turningStuff();
  differentialSteering();
  saveStatus();
#ifdef DEBUG
  delay(900);
#else
  delay(600);
#endif

#ifdef DEBUG
Serial.println();
Serial.print("heading to waypoint: ");
Serial.println(wp_hdg);
delay(300);
Serial.print("vessel heading: ");
Serial.println(data.heading);
delay(300);
Serial.print("distance to waypoint: ");
Serial.println(wp_dist);
#endif

}

