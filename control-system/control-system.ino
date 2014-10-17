/*
	control-system.ino

	This source file contains a skeleton control system, it is merely a starter point for the
	bottle boat control systems.
 */

//////////////////////////////////////////////////////////////////////////
/// External libraries
#include "Wire.h"
#include "Servo.h"
#include "math.h"

// code borrowed from demot
#define HMC6343_ADDRESS         0x19    // I2C address
#define HMC6343_HEADING_REG     0x50    // Bearing, pitch and roll register
#define rad2deg(x) (180/M_PI) * x
#define deg2rad(x) x * M_PI/180

Servo rudderServo;
int wp_hdg=0;
float wp_dist=0.0;
int wp_num=0;
float igain=0.01;
float pgain=0.1;
float running_err=0.0;
int hdg_err=0;

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

  // Now convert those siz bytes into a useful format using bit shifting. We have 3 values 
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
  //connect rudder (borrowing alot of code from demot :P)
  rudderServo.attach(5, 1060, 1920); // Attach, with the output limited (TODO, look up if these values can be improved)
  rudderServo.writeMicroseconds(1500); // Centre it roughly

}

void loop()
{

}
