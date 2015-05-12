// ---------------------------------------------------------------------------
// Created by Xavier Lagorce on 18/01/2014
// Copyright 2014 - Under creative commons license 3.0:
//        Attribution-ShareAlike CC BY-SA
//
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.
//
// Thread Safe: No
// Extendable: Yes
//
// @file I2CIO_soft.h
// This file implements a basic IO library using the PCF8574 I2C IO Expander
// chip with software I2C.
//
// @brief
// Implement a basic IO library to drive the PCF8574* I2C IO Expander ASIC through
// a sotfware I2C library provided by Bernhard Nebel and available here:
//  https://github.com/felias-fogg/SoftI2CMaster .
// The library implements basic IO general methods to configure IO pin direction
// read and write uint8_t operations and basic pin level routines to set or read
// a particular IO port.
//
// @version API 1.0.0
//
// @author X. Lagorce - Xavier.Lagorce@crans.org
// ---------------------------------------------------------------------------
#if (ARDUINO <  100)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#include <inttypes.h>


// Configure SoftI2CMaster library:
// SCL: pin 34 -> PC3
#define SCL_PIN 3
#define SCL_PORT PORTC
// SDA: pin 36 -> PC1
#define SDA_PIN 1
#define SDA_PORT PORTC
// Enable fast I2C mode
#define I2C_FASTMODE 1
// Include library
#include <../SoftI2CMaster/SoftI2CMaster.h>

#include "I2CIO_soft.h"

// CLASS VARIABLES
// ---------------------------------------------------------------------------


// CONSTRUCTOR
// ---------------------------------------------------------------------------
I2CIO_soft::I2CIO_soft ( )
{
   _i2cAddr     = 0x0;
   _dirMask     = 0xFF;    // mark all as INPUTs
   _shadow      = 0x0;     // no values set
   _initialised = false;
}

// PUBLIC METHODS
// ---------------------------------------------------------------------------

//
// begin
int I2CIO_soft::begin (  uint8_t i2cAddr )
{
   _i2cAddr = i2cAddr << 1; // Shift I2C address to comply with SoftI2CMaster

   _initialised = i2c_init();
   if (_initialised) {
      _initialised = i2c_start(_i2cAddr | I2C_READ);
      _shadow = i2c_read(true);
      i2c_stop();
   }

   return ( _initialised );
}

//
// pinMode
void I2CIO_soft::pinMode ( uint8_t pin, uint8_t dir )
{
   if ( _initialised )
   {
      if ( OUTPUT == dir )
      {
         _dirMask &= ~( 1 << pin );
      }
      else
      {
         _dirMask |= ( 1 << pin );
      }
   }
}

//
// portMode
void I2CIO_soft::portMode ( uint8_t dir )
{

   if ( _initialised )
   {
      if ( dir == INPUT )
      {
         _dirMask = 0xFF;
      }
      else
      {
         _dirMask = 0x00;
      }
   }
}

//
// read
uint8_t I2CIO_soft::read ( void )
{
   uint8_t retVal = 0;

   if ( _initialised )
   {
      _initialised = i2c_start(_i2cAddr | I2C_READ);
      retVal = ( _dirMask & i2c_read(true) );
      i2c_stop();
   }
   return ( retVal );
}

//
// write
int I2CIO_soft::write ( uint8_t value )
{
   int status = 0;

   if ( _initialised )
   {
      // Only write HIGH the values of the ports that have been initialised as
      // outputs updating the output shadow of the device
      _shadow = ( value & ~(_dirMask) );

      status = i2c_start(_i2cAddr | I2C_WRITE);
      i2c_write(_shadow);
      i2c_stop();
   }
   return ( status );
}

//
// digitalRead
uint8_t I2CIO_soft::digitalRead ( uint8_t pin )
{
   uint8_t pinVal = 0;

   // Check if initialised and that the pin is within range of the device
   // -------------------------------------------------------------------
   if ( ( _initialised ) && ( pin <= 7 ) )
   {
      // Remove the values which are not inputs and get the value of the pin
      pinVal = this->read() & _dirMask;
      pinVal = ( pinVal >> pin ) & 0x01; // Get the pin value
   }
   return (pinVal);
}

//
// digitalWrite
int I2CIO_soft::digitalWrite ( uint8_t pin, uint8_t level )
{
   uint8_t writeVal;
   int status = 0;

   // Check if initialised and that the pin is within range of the device
   // -------------------------------------------------------------------
   if ( ( _initialised ) && ( pin <= 7 ) )
   {
      // Only write to HIGH the port if the port has been configured as
      // an OUTPUT pin. Add the new state of the pin to the shadow
      writeVal = ( 1 << pin ) & ~_dirMask;
      if ( level == HIGH )
      {
         _shadow |= writeVal;

      }
      else
      {
         _shadow &= ~writeVal;
      }
      status = this->write ( _shadow );
   }
   return ( status );
}

//
// PRIVATE METHODS
// ---------------------------------------------------------------------------
