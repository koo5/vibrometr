/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/1231

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY
    
    v1.1 - Added Adafruit_Sensor library support
    v1.0 - First release
*/
/**************************************************************************/
#include "Arduino.h"

#include <Wire.h>
#include <limits.h>
#include <SPI.h>

#include "Adafruit_ADXL345_U.h"


void Adafruit_ADXL345_Unified::writeRegister(uint8_t reg, uint8_t value) {
  if (_i2c) {
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)(value));
    Wire.endTransmission();
  } else {
    digitalWrite(_cs, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(_cs, HIGH);
  }
}

uint8_t Adafruit_ADXL345_Unified::readRegister(uint8_t reg) {
  if (_i2c) {
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 1);
    return (i2cread());
  } else {
    reg |= 0x80; // read byte
    digitalWrite(_cs, LOW);
    SPI.transfer(reg);
    uint8_t reply = SPI.transfer(0);
    digitalWrite(_cs, HIGH);
    return reply;
  }  
}

/**************************************************************************/
/*! 
    @brief  Read the device ID (can be used to check connection)
*/
/**************************************************************************/
uint8_t Adafruit_ADXL345_Unified::getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
*/
/**************************************************************************/
Adafruit_ADXL345_Unified::Adafruit_ADXL345_Unified(int32_t sensorID) {
  _i2c = true;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class in SPI mode
*/
/**************************************************************************/
Adafruit_ADXL345_Unified::Adafruit_ADXL345_Unified(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t cs, int32_t sensorID) {
  _cs = cs;
  _i2c = false;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool Adafruit_ADXL345_Unified::begin() {

  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5)
  {
    Serial.print(F("not detected ... deviceid:"));
    Serial.println(deviceid, HEX);
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::setRange(range_t range)
{
  /* Read the data format register to preserve bits */
  uint8_t format = getRange();

  /* Update the data rate */
  format &= ~0x0F; //clear full_res, justify and range
  format |= range;
  
  /* FULL-RES enable  */
  format |= (1 << 3);


//invert interrupts (make them active low)
  format |= (1 << 5);


  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);
  
  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

range_t Adafruit_ADXL345_Unified::getRange(void)
{
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0b11);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t Adafruit_ADXL345_Unified::getDataRate(void)
{
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

#ifdef ssssssssssssssss

  event->acceleration.x = getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  sensor->max_value   = -156.9064F; /* -16g = 156.9064 m/s^2  */
  sensor->min_value   = 156.9064F;  /*  16g = 156.9064 m/s^2  */
  sensor->resolution  = 0.03923F;   /*  4mg = 0.0392266 m/s^2 */ 
#endif

HZ dataRate2hz(dataRate_t dataRate)
{
	return 320000/(1L<<(15-dataRate));
}

String hz_str(HZ hz)
{
  return String(hz) + String(F("/100Hz"));
}

String range_str(range_t r)
{
  return String(2 * (r+1)) + String("g");
}

void displayAccDetails(Acc &acc)
{
  Serial.println(F("ADXL345"));
  /*
  Serial.print  (F("Max: ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min: ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Res: ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  */
  Serial.print  (F("Range: +/- ")); Serial.println(range_str(acc.getRange()));
  Serial.print  (F("Rate:"));       Serial.println(hz_str(dataRate2hz(acc.getDataRate())));
  Serial.println("");
}


/*
#include <avr/pgmspace.h>
*/
