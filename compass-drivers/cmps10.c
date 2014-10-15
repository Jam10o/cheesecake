/*
	cmps10.c

	Author: Jordan
	Date: 14/10/2014

	This source file contains functions and definitions for using the CMPS10 compass. Copy 
	the contents of this file into your arduino sketch. You don't need to copy this comment 
	block. This code will not work standalone and goes hand in hand with the skeleton control
	system provide in the control-system folder

	Datasheet: http://www.robot-electronics.co.uk/htm/cmps10i2c.htm

	Wiring
		GND: Ground
		VCC: 3.3v
		SDA: A4
		SCL: A5
 */

#define CMPS10_ADDRESS		0x1E
#define CMPS10_REG_DATA		0x01

/**
 * Initialises the compass by first starting up I2C and then setting the compass's mode.
 */
void init_compass()
{
	// starts i2c
	Wire.begin();
	Serial.println("Started I2C");
	delay(50);
}

/**
 * Returns the forward heading of the compass
 */
int get_heading()
{
	byte msb, lsb;
	int heading;

	// set the register on the device we want to read.
	Wire.beginTransmission(CMPS10_ADDRESS);
	Wire.write(CMPS10_REG_DATA);
	Wire.endTransmission();

	// demand 6 bytes from the register we set earlier
	Wire.requestFrom(CMPS10_ADDRESS, 2);

	// here we wait for the 6 bytes and then convert them
	// into something usable using bit shifting
	while(Wire.available() < 2);
	msb = Wire.read();
	lsb = Wire.read();
	
	// get a heading in radians
	heading = (msb << 8) + lsb;

	// return the heading in degrees, 57.29582 = 180 / PI
	return heading / 10;
}