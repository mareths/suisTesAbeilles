/*
  Bmp180.h - Library for Barometric sensor.
  Created by S.Fret, October 17, 2016.
  Released into the public domain.
*/
#ifndef Bmp180_h
#define Bmp180_h

#include "Arduino.h"
#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

class Bmp180
{
  public:
    Bmp180();
    virtual ~Bmp180();
    void calibration();
    float getTemperature(unsigned int ut);
    float getPressure(unsigned long up);
    float calcAtm(float pressure);
    float calcAltitude(float pressure);
    unsigned int readUT();
    unsigned long readUP();
  private:
	// Calibration values
    int ac1;
	int ac2;
	int ac3;
	unsigned int ac4;
	unsigned int ac5;
	unsigned int ac6;
	int b1;
	int b2;
	int mb;
	int mc;
	int md;
	// b5 is calculated in getTemperature(...), this variable is also used in bmp085GetPressure(...)
	// so ...Temperature(...) must be called before ...Pressure(...).
	long b5;
	const unsigned char OSS = 0;  // Oversampling Setting
	// Private function
	char bmp085Read(unsigned char address);
	int bmp085ReadInt(unsigned char address);
	void writeRegister(int deviceAddress, byte address, byte val);
	int readRegister(int deviceAddress, byte address);
};

#endif