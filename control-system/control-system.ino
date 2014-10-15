/*
	control-system.ino

	This source file contains a skeleton control system, it is merely a starter point for the
	bottle boat control systems.
 */

//////////////////////////////////////////////////////////////////////////
/// Externel libraries
#include "Wire.h"


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

}

void loop()
{

}