//////////////////////////////////////////////////////////////////////////
/// External libraries
#include "Wire.h"
#include "Servo.h"
#include "math.h"
#include "TinyGPS.h"
#include "Time.h"
#include "SoftwareSerial.h"

// lots of code borrowed from demot
#define HMC6343_ADDRESS         0x19    // I2C address
#define HMC6343_HEADING_REG     0x50    // Bearing, pitch and roll register
#define rad2deg(x) (180/M_PI) * x
#define deg2rad(x) x * M_PI/180
#define GPS_ENABLE_PIN          10
#define NUM_OF_WAYPOINTS 8
#define WP_THRESHOLD 5
float wp_lats[NUM_OF_WAYPOINTS]; 
float wp_lons[NUM_OF_WAYPOINTS];

//
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
  uint16_t wind_dir;
  int8_t roll;
  int8_t pitch;
  int8_t rudder;
  float lat;
  float lon;
  long unix_time;
} data;


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
  byte buf[6];

  Wire.beginTransmission(HMC6343_ADDRESS); // Start communicating with the HMC6343 compasss
  Wire.write(HMC6343_HEADING_REG); // Send the address of the register that we want to read
  Wire.write(0x55); // Send the address of the register that we want to read
  Wire.endTransmission();
  Wire.requestFrom(HMC6343_ADDRESS, 6); // Request six bytes of data from the HMC6343 compass, 
                                        // if you look at the data sheet you will see that the 
                                        // data we want is stored in a 6 byte register
  for(int i=0;i<6;i++)
  {
    while(Wire.available() < 1); // Busy wait while there is no byte to receive
    buf[i]=Wire.read();
    //printf("buf[%d]=%d\r\n",i,buf[i]);
  }

  // Now convert those six bytes into a useful format using bit shifting. We have 3 values 
  // stored in 6 bytes, so 2 bytes per a value. If you don't understand whats quite going on
  // here thats perfectly fine. Hopefully you will by the end of the year :)
  int heading = ((buf[0] << 8) + buf[1]); // the heading in degrees
  int pitch =   ((buf[2] << 8) + buf[3]); // the pitch in degrees
  int roll = ((buf[4] << 8) + buf[5]); // the roll in degrees*/

  // Bring our values in the 0-359 range
  heading=heading/10;
  roll=roll/10;
  pitch=pitch/10;
  data.roll=(int8_t)roll;
  data.pitch=(int8_t)pitch;
  data.heading=(uint16_t)heading;

  delay(100);

  return (int)heading; // Print the sensor readings to the serial port.
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

void differentialSteering(int head) {
}  

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
  
///////////////////////////////////////////////////////////////////////////////////
/// Here we setup waypoint stuff, these are actually the coordinates of the place
/// Dermot was actually meant to race in at WRSC in Galway last summer
// start (TODO, change these to the values we need)
wp_lats[0] = 53.257804;
wp_lons[0] = -9.117945;

wp_lats[1] = 53.257795;
wp_lons[1] = -9.117450;

wp_lats[2] = 53.257805;
wp_lons[2] = -9.117440;

wp_lats[3] = 53.257918;
wp_lons[3] = -9.117673;

wp_lats[4] = 53.257928;
wp_lons[4] = -9.117683;

wp_lats[5] = 53.257814;
wp_lons[5] = -9.117955;

wp_lats[6] = 53.257805;
wp_lons[6] = -9.117440;
// End / Home
wp_lats[7] = 53.25851;
wp_lons[7] = -9.11918;
///////////////////////////////////////////////////////////////////////////////////

}

void loop()
{
  orientationStuff();
  int diffsteerhead = turningStuff();
  differentialSteering(diffsteerhead);
  saveStatus();
}
