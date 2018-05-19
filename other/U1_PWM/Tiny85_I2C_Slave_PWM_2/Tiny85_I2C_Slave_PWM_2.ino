// Steve Quinn 31/12/16
//
// Copyright 2016 Steve Quinn
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// Compiled using Arduino 1.6.9, Tested with ATTiny85, debugged with Arduino Uno R3
//
// Uses the following libraries;
// QuarticEase by Andy Brown : http://andybrown.me.uk/2010/12/05/animation-on-the-arduino-with-easing-functions/
// ATTiny Core By Spence Konde : https://github.com/SpenceKonde/ATTinyCore/blob/master/README.md
// TinyWireS By Eero af Heurlin : https://github.com/rambo/TinyWire/tree/master/TinyWireS
//
// ATtiny85 acts as a slave I2C device (address 0x08). When sent a single hex control byte 0 ... 255 this is forwarded to PWM1 
// on the device. Forwarding of the new control byte is done in increments via the use of an easing function such that if it 
// were driving an led or an LCD backlight there is a smooth transition from one light level to the next. As this is achieved 
// autonomously there is no processing burden on the control microprocessor other than to send the new light level over I2C.
//
// ATMEL ATTINY85 / ARDUINO Pin connections
//
//                           +-\/-+
//            N/C (nRESET)  1|    |8   VCC (2.7-5.5V)
//            N/C    (D 3)  2|    |7   PI2C SCL
//            N/C    (D 4)  3|    |6   (D 1) PWM1 O/P
//                     GND  4|    |5   I2C SDA
//                           +----+
//
// Note : 1. When using the Arduino Uno R3 to debug, the output led should be connected to the PWM pin 9.
//        2. If this is the only device on your I2C bus, don't forget to add 10K pull up resistors to SDA and SCL lines
//           See : http://www.instructables.com/id/Arduino-I2C-LCD-Driver-Library-and-PackMan/ : I2C_LCD_With_Arduino.pdf

//#define ARDUINO_UNO_DEBUG               // Define this to debug code on the Arduino Uno
//#define ARDUINO_UNO_EASING_LOOP_DEBUG   // Define this to get debug on easing loop. Be careful it'll generate a lot of data, MAX_EASING_COUNTS per command. ARDUINO_UNO_DEBUG needs to be defined to use this

#if defined (ARDUINO_AVR_UNO) 
  #if defined(ARDUINO_UNO_DEBUG)
    #define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
  #endif
  #include <Wire.h>
#else
  #include "TinyWireS.h"                  // wrapper class for I2C slave routines
#endif
//#include "ExponentialEase.h"
#include "QuarticEase.h"

extern "C" void __cxa_pure_virtual() { }


#if defined (ARDUINO_AVR_UNO) 
#define PWM1               9            // Arduino Uno Pin 9
#else
#define PWM1               1            // ATtiny Pin 6
#endif
#define I2C_SLAVE_ADDR  0x08            // i2c slave address 
#define PWM_OFF         0x00            // Const to turn PWM ouput off

//ExponentialEase ease;
QuarticEase ease;
double  dEasedPosition = ((double)0.0);
double  dEasingInterval = ((double)0.0);
bool    bNewValue = false;
bool    bEasingInProgress = false;
int     iEasingCounts = 0;
#define MAX_EASING_COUNTS         ((int)100)
#define EASING_INTERVAL_INCREMENT ((double)0.02)
int     iDelta = 0;
int     iSign = 1;
byte    bRcvd_new = 0;
byte    bRcvd_old = 0;
byte    bCurrentValue = 0;
unsigned long previousMillis = 0; // previous time value, so routine is non blocking
const long delayInterval = 15    ;  // interval at which to make a new easing calculation, in milli seconds
double  dTmpCurrentValue = 0.0;

void setup(){
  ease.setDuration(2.0);
  ease.setTotalChangeInPosition(255.0);  
  pinMode(PWM1,OUTPUT);                 // Led output pin
  analogWrite(PWM1, PWM_OFF);           // ensure output is low
  #if defined (ARDUINO_AVR_UNO) 
    Wire.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode
    Wire.onReceive(receiveEvent);   // register event
    #if defined(ARDUINO_UNO_DEBUG)
    Serial.begin(115200);
    Serial.println(__FILENAME__);
    Serial.println("Serial Port Ready");
    #endif
  #else
    TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode
  #endif
}



#if defined (ARDUINO_AVR_UNO) 
void receiveEvent(int howMany) {
  if (howMany > 0) {
    bRcvd_new = Wire.read();
    if (bRcvd_old != bRcvd_new){
      if (bEasingInProgress) 
      {
        bCurrentValue = ((byte)dTmpCurrentValue);
        bEasingInProgress = false;
      }
      bNewValue = true;
      #if defined(ARDUINO_UNO_DEBUG)
      Serial.println("New Val Detected");
      #endif
    } else 
    #if defined(ARDUINO_UNO_DEBUG)
    Serial.println("Same Val Detected");
    Serial.print("bRcvd_new : ");
    Serial.println(bRcvd_new);
    Serial.print("bRcvd_old : ");
    Serial.println(bRcvd_old);
    #else
    ;
    #endif
  }
}
#endif

void loop(){
  unsigned long currentMillis = millis();

  #if defined (ARDUINO_AVR_UNO)
  if (false){           // got I2C input!
  #else
  if (TinyWireS.available()){           // got I2C input!
  #endif
    #ifndef ARDUINO_AVR_UNO
    bRcvd_new = TinyWireS.receive();     // get the byte from master
    #endif
    if (bRcvd_old != bRcvd_new)
    {
      if (bEasingInProgress) 
      {
        bCurrentValue = ((byte)dTmpCurrentValue);
        bEasingInProgress = false;
      }
      bNewValue = true;
    }  
  }

  if (bNewValue)
  {
    #if defined (ARDUINO_AVR_UNO) && defined(ARDUINO_UNO_DEBUG)
    Serial.println("Got NV");
    #endif
    bNewValue = false;
    iDelta = (int) (((int)bCurrentValue) - ((int)bRcvd_new));
    if (iDelta < 0) {
      iSign = 1;
      iDelta *= -1;
    } else
      iSign = -1;
      
    #if defined (ARDUINO_AVR_UNO) && defined(ARDUINO_UNO_DEBUG)
    Serial.print("iDelta : ");
    Serial.println(iDelta);
    Serial.print("iSign : ");
    Serial.println(iSign);
    #endif
      
    ease.setTotalChangeInPosition((double)iDelta);      
    bRcvd_old = bRcvd_new;
    previousMillis = millis();
    currentMillis = previousMillis;
    iEasingCounts = 0;
    dEasingInterval = 0.0;
    dTmpCurrentValue = 0.0;
    dEasedPosition = 0.0;
    bEasingInProgress = true;
  }

  if (bEasingInProgress) 
  {
    if(currentMillis - previousMillis >= delayInterval) {
      previousMillis = currentMillis;   
      
      dEasedPosition   = ease.easeInOut(dEasingInterval);
      dEasingInterval += EASING_INTERVAL_INCREMENT;
      dEasedPosition  *= ((double)iSign);
      
      dTmpCurrentValue = ((double)bCurrentValue) + dEasedPosition;
      
      if (dTmpCurrentValue < 0.0)
        bCurrentValue = 0;
      else
        if (dTmpCurrentValue > 255.0)
          dTmpCurrentValue = 255;
          
      #if defined (ARDUINO_AVR_UNO) && defined(ARDUINO_UNO_EASING_LOOP_DEBUG)
      Serial.print("dTmpCurrentValue : ");
      Serial.println(dTmpCurrentValue);
      #endif

      analogWrite(PWM1, ((byte)dTmpCurrentValue));

      iEasingCounts++;
      if (iEasingCounts >= MAX_EASING_COUNTS)     
      {
        bEasingInProgress = false;
        
        bCurrentValue = ((byte)dTmpCurrentValue);
        #if defined (ARDUINO_AVR_UNO) && defined(ARDUINO_UNO_DEBUG)
          Serial.println("bEasingInProgress = false");
          Serial.print("bCurrentValue : ");
          Serial.println(bCurrentValue);
        #endif
      }
    }
  }
}

 
 

