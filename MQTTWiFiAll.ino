// Copyright 2018 Steve Quinn
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// For ESP8266-12E
// Requires MQTT PubSubClient Library found here:                 https://github.com/knolleary/pubsubclient
// Requires BounceI2C Library found here:                         https://github.com/SteveQuinn1/BounceI2C
// Requires Adafruit DHT Library found here:                      https://github.com/adafruit/DHT-sensor-library
// Requires DS1307RTC Library found here:                         https://github.com/PaulStoffregen/DS1307RTC
// Requires Time Library found here:                              https://github.com/PaulStoffregen/Time
// Requires BMP085 Library found here:                            https://github.com/adafruit/Adafruit-BMP085-Library
// Requires LiquidCrystal_I2C_PCF8574 Library found here:         https://github.com/SteveQuinn1/LiquidCrystal_I2C_PCF8574
// Requires BH1750 Library found here:                            https://github.com/claws/BH1750
// Requires APDS9960 RGB and Gesture Sensor Library found here:   https://github.com/sparkfun/SparkFun_APDS-9960_Sensor_Arduino_Library/tree/V_1.4.2
// Requires ESP8266WiFi Library found here:                       https://github.com/ekstrand/ESP8266wifi/
// Requires Arduino IDE support for ESP8266 found here:           http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// Requires ESP8266WebServer Library found here:                  http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// Requires ESP8266mDNS Library found here:                       http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// For ATMega328P
// Requires APDS9960_NonBlocking Library found here:              https://github.com/SteveQuinn1/APDS9960_NonBlocking
// Requires Modified SparkFun APDS9960 Library found here:        https://github.com/SteveQuinn1/SparkFun_APDS-9960_Sensor_Arduino_Library
//
// Overview of software
// --------------------
// Uses DHT22 (Temp and Humidity), DS1307Z (Real Time Clock), BMP085 (Barometric Sensor), BH1750 (Ambient Light Sensor), 
// APDS9960 (RGB and Gesture Sensor) indirectly, NXP PCF8574 General I/O and LCD I/O
// Logs csv files to SD Card via SPI interface detailing CSV Filename, displays summary data on a 20 b7 4 LCD, Barometric pressure,
// Temperature and Humidity, Time and Date 
// Utilises two buttons to open and close logging file, reads two button states, outputs to two leds.
//
// Compiled for ESP8266-12E on Arduino IDE 1.6.9
// 
// Inclusive of the above the software supports MQTT over WiFi and allows for the following;
// 1. Setting of Time and Date
// 2. Opening and closing of csv log file
// 3. Setting and reading logging status
// 4. Getting and Setting logging period
// 5. Reading button status
// 6. Setting led statuses
// 7. Reading software version
// 8. Reading local, Temperature, Humidity, Heat Index, Barometric pressure, Ambient light level.
// 9. Limited recognition of gestures. Up/Down/Left/Right/Near/Far
//
//
// Maintenance History
// 16/03/17 : 2      Inherits from ESP8266TempHumi7_17_5.ino (stable IoT framework) and MQTTEthernetAll6_6.ino (data logging framework). First creation of file.
// 18/03/17 : 3      This compiles, but have removed all SPIFFS as it can't coexist with SD. Edited out SPIFFS format command as there is nothing similar for SD.
// 18/03/17 : 4      Got SD Card access working
// 18/03/17 : 5      Added button status (WIP), added callback handlers for Barometric zero and offset, added configruation file capability. Question mark over reporting strategy for now. Working ALS and backlight delta
// 19/03/17 : 6      Updated logging control, added string parser to replace sscanf
// 25/03/17 : 6_1    Updated string parser, corrected SD Card 'fileWrite' append error.
// 25/03/17 : 6_1_1  Updated string parser, corrected SD Card 'fileWrite' append error. Specific version to test application of yield() to see if stablity is improved.
// 25/03/17 : 8      Picked up changes in 6_1_1 and merged loggin code from 7. Added in local logging to file but this is still unstable and causes WDT reboot.
// 25/03/17 : 9      Added ALS value send, controlled by RepStrat, modified checkTemp function to send last good reading if DHT read fails for a none zero RepStrat
// 26/03/17 : 10     This version did fail after around 24hrs.
// 27/03/17 : 11     Added linked list handler for holding Sensor Instances. Still a bit ropey, can't figure out why commented code won't exe. Compiler issue, see community boards?
// 01/04/17 : 12     Display scrolling now working, linked list fine, reads in from consenin.txt no problems, no reboots pretty stable over time. Need to tidy up how to handle no consenin.txt present
// 01/04/17 : 13     Screen scrolls displays LOCAL_TEMP_AND_HUMIDITY_STRING if no consenin.txt file present or no linked list. Added sensorListGetPrevious()
// 01/04/17 : 14     Added callback handling for MQTT Temp and Humi logging. Display now cycling through T&H for all configured sensors
// 08/04/17 : 16     Added functionality to scroll display on hand gesture
// 14/04/17 : 16_1   Added interrupt handling on gesture controller. Gesture now working. Needed to modify the 'SparkFun APDS9960 Gesture Library' also added control var to switch off gesture control from SD config param
// 14/04/17 : 16_2   Added configurable Up/Down gesture control.
// 15/04/17 : 16_3   Added button control via I2C NXP_PCF8574P, adapted Bounce2 library to work via I2C.
// 16/04/17 : 16_4   Got logging functionality working, stupid error in variable parameter string with sprintf. I request 5 and supplied 4. fixed local logging if temp and humid. Fixed logging buttons, can only start if stopped, can only stop if started. System can be temperamental at start up. I suspect it's the loading of the I2C 
// 16/04/17 : 16_5   Added configurable button capability. This works Fine.
// 17/04/17 : 16_6   Added extra input button
// 21/04/17 : 16_7   Added trending arrow for barometric pressure
// 23/04/17 : 16_8   Added extra LED.
// 27/12/17 : 16_9   Debug LCD build.
// 27/12/17 : 16_9_1 Added code to filter out RTC random time date strings "45:165" "165/165/165".
// 10/03/18 : 16_9_2 Removed gesture sensor code to relocate to ATMega328P-PU
// 10/03/18 : 16_9_3 Added code to make alternative Temperature/Humidity readings. Gives a more reliable performance.
// 29/04/18 : 16_9_4 Inverted Leds due to PCB implementation. Also swapped up/down, left/right for the gesture sensor due to it's mounting orientation and corrected button inputs inline with wiring loom.
// 06/05/18 : 16_9_5 Added code to make button reads more robust, 
//                   Added topic to allow the reading of the contents of the system LCD to enable automated testing,
//                   Added topic to allow setting of a given Configurable Sensor on the system LCD to enable automated testing,
//                   Added logic to handle sensor failure for Barometer and Gesture Sensors.
//
// Start up sequence
// -----------------
// Unit starts up in STA_AP mode and looks for SD file SECURITY_PARAMETERS_FILE. If the file doesn't exist the IoT device switches to AP mode, 
// starts a web server and initialises an mDNS service. The device then broadcasts the SSID AP_NETWORK_SSID_DEFAULT + DeviceMACAddress. To connect 
// to this APnetwork use the following password AP_NETWORK_PASSWORD_DEFAULT once connected enter the following URL into your browser nDNSHostName + '.local'
// The IoT device will then serve up a configuration web page allowing new sensor network security parameters to be submitted.
// If the SD file is present the information in this file is used by the IoT device to connect to the sensor network. The device then stays in STA_AP 
// mode until it has connected. Once connected to the sensor network the IoT device switches to STA mode.
// If the information in the SECURITY_PARAMETERS_FILE does not allow connection to the sensor network it is possible to connect to the sensors APnetwork as above,
// only now you must enter the IP address 192.168.4.1. This is due to a flaw in the mDNS library of the ESP8266. When in STA_AP mode mDNS service
// will not work.
// Once the device has connected to the sensor network it attempts to connect to the MQTT broker which it expects at the following address MQTT_BROKER_IP_DEFAULT
// and port MQTT_BROKER_PORT_DEFAULT. If the IoT device exceeds mqtt_broker_connection_attempts it will re-initialise as if no SECURITY_PARAMETERS_FILE were present.
// Once connected to the MQTT broker, if the connection to the broker is lost the system re-initialises.
// If mqtt_broker_connection_attempts=0 the IoT device will continue to attempt an MQTT Broker connection.
//
// To give a visual indication of the above connection states, the IoT device will flash the local led as follows.
// 1. When no onboard configuration file is present SECURITY_PARAMETERS_FILE 1 quick flash.
// 2. When attempting to connect to a given WiFi network 2 quick flashes in succession.
// 3. Once a WiFi n/w connection has been achieved. Whilst attempting to connect to an MQTT Broker the led will be on continuously.
// 4. Once WiFi n/w and MQTT Broker connections are in place the led will extinguish.
//
// WiFi and IoT Broker connections are also indicated on the LCD, as is Access Point availability.
//
// Configuration files
// Here there are five text files named 'calvals.txt', 'secvals.txt', 'confvals.txt', 'consenin.txt', 'congesud.txt', 'conbutin.txt' and 'index.htm' + some other crap such that a little 'IoT' icon appears in the browser address bar.
// Copy these to the root directory of the system SD card.
//
// 'calvals.txt' contains seven entries. These values are exposed for read write via MQTT topics.
// - 1st Calibration zero offset for Temperature a float
// - 2nd Calibration scale factor for Temperature a float
// - 3rd Calibration zero offset for Humidity a float
// - 4th Calibration scale factor for Humidity a float
// - 5rd Calibration zero offset for Barometric Pressure a float
// - 6th Calibration scale factor for Barometric Pressure a float
// - 7th Value is the reporting strategy value. 0 = Send and update when a change is detected in the monitored value. 1...60 = report back the monitored value every 'n' minutes
//
// 'secvals.txt' contains six entries. These values are write only via MQTT topics.
// - 1st MQTT Broker IP Address. In dotted decimal form AAA.BBB.CCC.DDD
// - 2nd MQTT Broker Port. In Integer form.
// - 3rd MQTT Broker connection attempts to make before switching from STA mode to AP mode. In Integer form. 
// - 4th WiFi Network SSID. In free form text.
// - 5th WiFi Network Password. In free form text.
// - 6th WiFi Network connection attempts to make before switching from STA mode to AP mode. In Integer form. // NOTE this is not implemented
//
// 'confvals.txt' contains seven entries. Used to control the automatic switching of the display back light depending upon ambient room lighing levels
// - 1st Display Ambient Light Sensor Upper Threshold Value. In Unsigned Long form.
// - 2nd Display Ambient Light Sensor Lower Threshold Value. In Unsigned Long form.
// - 3rd Display Delay Before Change Value. In Unsigned Long form.
// - 4th Display Backlight Upper Value. In Unsigned Long form.
// - 5th Display Backlight Lower Value. In Unsigned Long form.
// - 6th Display Delay Before Scroll Value. In Unsigned Long form.
// - 7th Enable gesture control. In boolean form 0 = GC off, 1 = GC on
//
// 'consenin.txt' contains configurable entries which enable the unit to subscribe to two topics Temp and Humidity and link these topics to a logical area
// such as bedroom, office, garage etc. The unit then monitors MQTT traffic for this sensor data and updates a local copy. This local copy is then scrolled
// on the system LCD display along with the logical area. The only limit to the number of entries is available system ram.
// - 1st : Logical name. ie Garage
// - 2nd : Temperature Topic. ie. WFD/THSen/1/TempStatus/1
// - 3rd : Humidity Topic. ie. WFD/THSen/1/HumdStatus/1
// - REPEAT PATTERN
//
// 'congesud.txt' contains configurable gesture entries which are matched to Up and Down gestures respectively. The file must contain 4 entries as below.
// The entries are in the form of MQTT Topics and their respective payloads.
// - 1st : Up Gesture Topic. ie. /OHB/Req/StudyLights/Control
// - 2nd : Up Gesture Payload. ie. 1
// - 3rd : Down Gesture Topic. ie. /OHB/Req/StudyLights/Control
// - 4th : Down Gesture Payload. ie. 0
//
// 'conbutin.txt' contains configurable button entry. The file must contain 6 entries as below.
// The entries are in the form of MQTT Topics and their respective payloads. If an entry is not required then the key word 'null' must be placed in the Topic and Payload position
// For button 0 the first Payload is published on being presses and the second on being released. 
// For button 1 the Payload is published when the button is pressed
// - 1st : Button0 Topic. ie. /OHB/Req/StudyLights/Control
// - 2nd : Button0 Payload. ie. 1
// - 3rd : Button0 Topic. ie. /OHB/Req/StudyLights/Control
// - 4th : Button0 Payload. ie. 0
// - 5th : Button1 Topic. ie. /OHB/Req/StudyLights/Control
// - 6th : Button1 Payload. ie. 0
//
// 'index.htm'
// Contains web page served up when the device can't connect to the Network using the password held in the 'secvals.txt' file
//
// Arduino IDE programming parameters.
// 
// From Tools Menu
// Board: 'Generic ESP8266 Module'
// Flash Mode: 'DIO'
// Flash Size: '4M (3M SPIFFS)'
// Debug Port: 'Disabled'
// Debug Level: 'None'
// Reset Method: 'ck'
// Flash Frequency '40MHz'
// CPU Frequency '80 MHz'
// Upload Speed: '115200'
// 

   
//#define DEBUG_GENERAL      // Undefine this for general debug information via the serial port. Note, you must deploy with this undefined as your network security parameters will be sent to serial port
//#define DEBUG_WEB          // Undefine this for comprehensive debug information on web interface via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_MDNS         // Undefine this for comprehensive debug information on mDNS support via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_TIMER        // Undefine this for timer debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_SD           // Undefine this for SD debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_VALIDATION   // Undefine this for validation debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_STATE_CHANGE // Undefine this for 'eSENSORSTATE' State Change debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_LEDFLASH     // Undefine this for 'eLEDFLASHSTATE' State Change debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_SECVALS      // Undefine this for MQTT SecVals Topic debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_PARMGRAB     // Undefine this for parmGrab function debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_BACKLIGHT    // Undefine this for backlight brightness/dimming function debug information. Requires DEBUG_GENERAL. Can generate a lot of output.
//#define DEBUG_LOGGING      // Undefine this for logging function debug information. Requires DEBUG_GENERAL.
//#define DEBUG_PARSER       // Undefine this for logging StringParser function. Requires DEBUG_GENERAL.  
//#define DEBUG_LCD_UPDATE   // Undefine this for debugging display scroll functionality. Requires DEBUG_GENERAL.  
//#define DEBUG_DT           // Undefine this for debugging date time functionality. Requires DEBUG_GENERAL.  
//#define DEBUG_GESTURE      // Undefine this for debugging gesture functionality. Requires DEBUG_GENERAL.  
//#define DEBUG_BUTTONS      // Undefine this for debugging button functionality. Requires DEBUG_GENERAL.  
//#define DEBUG_BARO_TREND   // Undefine this for debugging Barometric Trending functionality. Requires DEBUG_GENERAL.  
//#define DEBUG_BARO_FAKE_DATA // Undefine this for debugging Barometric Trending functionality with faked Barometric data. Requires DEBUG_GENERAL and DEBUG_BARO_TREND.  





//#define FS_NO_GLOBALS
#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <BounceI2C.h>
#include <LiquidCrystal_I2C_PCF8574.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include <stdio.h>
#include <stdarg.h>
#include <malloc.h>
#include <APDS9960_NonBlocking.h>



#define TO_UPPER 0xDF
#define MAKE_UPPER_CASE(c)     ((c) & TO_UPPER)
#define SET_BIT(p,whichBit)    ((p) |=  (1    << (whichBit)))
#define CLEAR_BIT(p,whichBit)  ((p) &= ~((1)  << (whichBit)))
#define TOGGLE_BIT(p,whichBit) ((p) ^=  (1    << (whichBit)))
#define BIT_IS_SET(p,whichBit) ((p) &   (1    << (whichBit)))

#define SD_FILE_READ_MODE  FILE_READ
//#define SD_FILE_WRITE_MODE FILE_WRITE
#define SD_FILE_WRITE_MODE (O_WRITE | O_CREAT | O_TRUNC)


#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

// Define I2C pins on ESP8266-12E
#define I2C_SDA       4               // s/w defined I2C lines on the ESP8266-12E
#define I2C_SCL       5               // s/w defined I2C lines on the ESP8266-12E

// Instantiate system LCD display
#define MAX_SYSTEM_LCD_COLUMNS 20
LiquidCrystal_I2C_PCF8574 lcd(0x27,MAX_SYSTEM_LCD_COLUMNS,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

// LCD Display custom characters
#define WiFiOff 0  
#define WiFiOn 1  
#define MQTTOff 2 
#define MQTTOn 3  
#define AccessPointOn 4
#define Degrees 5
#define BaroUp 6
#define BaroDown 7
 
typedef struct {  
  const unsigned char array1[8];
} binaryArrayType;  
  
binaryArrayType binaryArray[8] =  
{ 
  {0x11,0x0A,0x04,0x0A,0x11,0x04,0x04,0x04},  // WiFiOff, 0
  {0x1F,0x00,0x0E,0x00,0x04,0x00,0x04,0x04},  // WiFiOn, 1
  {0x11,0x0A,0x04,0x0A,0x11,0x0E,0x04,0x04},  // MQTTOff, 2
  {0x00,0x17,0x15,0x17,0x00,0x0E,0x04,0x04},  // MQTTOn, 3
  {0x1C,0x14,0x1C,0x14,0x07,0x05,0x07,0x04},  // AccessPointOn, 4
  {0x07,0x05,0x07,0x00,0x00,0x00,0x00,0x00},  // Degrees, 5
  {0x04,0x0A,0x15,0x04,0x04,0x04,0x04,0x04},  // BaroUp, 6
  {0x04,0x04,0x04,0x04,0x04,0x15,0x0A,0x04}   // BaroDown, 7
};  



#define CALIBRATION_PARAMETERS_FILE             ((char *)("/calvals.txt"))
#define SECURITY_PARAMETERS_FILE                ((char *)("/secvals.txt"))
#define CONFIGURATION_PARAMETERS_FILE           ((char *)("/confvals.txt"))
#define CONFIGURABLE_SENSOR_INPUT_FILE          ((char *)("/consenin.txt"))
#define CONFIGURABLE_UP_DOWN_GESTURE_INPUT_FILE ((char *)("/congesud.txt"))
#define CONFIGURABLE_BUTTON_INPUT_FILE          ((char *)("/conbutin.txt"))
#define LOWER_REPORTING_STRATEGY_VALUE          0  // Report any change
#define UPPER_REPORTING_STRATEGY_VALUE          60 // Send updates every hour
#define TEMPERATURE_CALIBRATION_OFFSET_DEFAULT  ((float)0.0)
#define TEMPERATURE_CALIBRATION_SCALE_DEFAULT   ((float)1.0)
#define HUMIDITY_CALIBRATION_OFFSET_DEFAULT     ((float)0.0) 
#define HUMIDITY_CALIBRATION_SCALE_DEFAULT      ((float)1.0) 
#define BAROMETRIC_CALIBRATION_OFFSET_DEFAULT   ((float)0.0) 
#define BAROMETRIC_CALIBRATION_SCALE_DEFAULT    ((float)1.0) 
#define REPORTING_STRATEGY_DEFAULT              LOWER_REPORTING_STRATEGY_VALUE
#define DEFAULT_SENSOR_INTERVAL                 10000 // Sensor read interval in Milliseconds
#define DEFAULT_TH_SENSOR_INTERVAL              5000  // Temperature/Humidity Sensor read interval in Milliseconds

// Instantiate BMP085 Barometric Sensor
Adafruit_BMP085 bmp;

// Values read from Barometric sensor
boolean bBMPSensorFail    = false;                                  // Records if Barometric sensor has failed at start up
boolean newBarometricData = false;                                  // Flag to trigger display update on change of barometric data
char strTempBaro[20+1]    = "0000.00";                              // String to hold barometric value
char strTempBaroOld[20+1] = "0000.00";                              // String to hold barometric value
unsigned long barometric_old = 0;                                   // Previous Barometric reading
unsigned long barometric_new = 0;                                   // New Barometric reading
unsigned long previousBarometricMillis = 0;                         // previous time value, so routine is non blocking
unsigned long readIntervalBarometric = DEFAULT_SENSOR_INTERVAL;     // interval at which to read sensor, in milli seconds (10secs)
float baroCalOffset      = BAROMETRIC_CALIBRATION_OFFSET_DEFAULT;
float baroCalScale       = BAROMETRIC_CALIBRATION_SCALE_DEFAULT;

#ifdef DEBUG_BARO_TREND
#define MAX_BAROMETRIC_SAMPLES 3
#else
#define MAX_BAROMETRIC_SAMPLES 3
#endif
#define MAX_BAROMETRIC_DELTA 0.03
#define BAROMETRIC_DELTA_UPPER 3
#define BAROMETRIC_DELTA_LOWER -3
float fBarometricRollingAverage[MAX_BAROMETRIC_SAMPLES];
boolean bFirstBarometricRead = true;
int iBarometricRollingAveragePointer = 0;
int iBarometricDeltaCount = 0;
boolean sendBTrendUpdate = false;
typedef enum {
   eRISING  = 0,
   eSTABLE  = 1,
   eFALLING = 2
} eBAROMETRIC_PRESSURE_STATE;

float fOldBarometricTrendAverage = 0.0;
float fNewBarometricTrendAverage = 0.0;

#ifdef DEBUG_BARO_FAKE_DATA
unsigned long  ulDummyBarometricTrendArray[] = {100000, 100000, 100000, 99500, 99000, 98500, 98000, 97500, 97000, 96500, 96000, 96000, 96000, 96000, 96000, 96000, 95500, 96000, 96500, 97000, 98000, 99000};
int iDummyBarometricTrendArrayPointer = 0;
#endif

// For RTC DS1307Z
#define TOTAL_TIME_IN_MINUTES(H, M) ((H*60)+(M))
#define CONVERT_TO_MINUTES(x) ((x.Hour*60)+(x.Minute))
#define TWENTYFOUR_HOURS_IN_MINUTES (24 * 60)
tmElements_t tm;
tmElements_t tm_old;
const char strDateFormat[] = "DD/MM/YY";
const char strTimeFormat[] = "HH:MM";
char tmpDateStrOld[20+1] = "DD/MM/YY";
char tmpTimeStrOld[20+1] = "HH:MM";
boolean bNewTimeDateSet = false;


// Sensor Instances Linked List Handler
#define MAX_SENSOR_NAME_STRING (MAX_SYSTEM_LCD_COLUMNS + 1)
#define MAX_TOPIC_STRING       (MQTT_MAX_PACKET_SIZE   + 1)
#define LOCAL_TEMP_AND_HUMIDITY_STRING "Local T&H"
const char defaultTempStr[] = "tt.tt";
const char defaultHumidStr[] = "hh.hh";
const char defaultHICStr[] = "tt.tt";


typedef struct SensorInstance {
    char   strSensorName[MAX_SENSOR_NAME_STRING];
    char   strTemperatureTopic[MAX_TOPIC_STRING];
    char   strHumidityTopic[MAX_TOPIC_STRING];
    char   strTemperature[(sizeof(defaultTempStr)/sizeof(char))+1];
    char   strHumidity[(sizeof(defaultHumidStr)/sizeof(char))+1];
    boolean bExternalSource;
    struct SensorInstance *next;
    struct SensorInstance *previous;
} sSensorInstance;



// Values used to control display scrolling
#define DISPLAY_DELAY_BEFORE_SCROLL_VALUE_DEFAULT   ((unsigned long) (10000))  // In mS
unsigned long displayDelayBeforeScrollValue = DISPLAY_DELAY_BEFORE_SCROLL_VALUE_DEFAULT;

sSensorInstance *ptrHeadOfSensorInstances = NULL;
sSensorInstance *tmpSensorInstancePtr     = NULL;
boolean bScrollDisplay = false;
boolean bNewTemperatureHumidityData       = false;  // Used to trigger a T&H display update if no CONFIGURABLE_SENSOR_INPUT_FILE is present or linked list could not be created
boolean bOneOff                           = false;  // Triggers a one time write to the system LCD to write the static name of this device on the display 'LOCAL_TEMP_AND_HUMIDITY_STRING'

// Values read from BH1750FVI I2C Ambient Light Intensity Sensor and backlight values
BH1750 ambientLightSensor;
#define BACKLIGHT_I2C_ADDRESS                   0x08
#define DISPLAY_BACKLIGHT_UPPER_VALUE_DEFAULT   ((unsigned long) (255))
#define DISPLAY_BACKLIGHT_LOWER_VALUE_DEFAULT   ((unsigned long) (10))
#define DISPLAY_BACKLIGHT_DEFAULT_VALUE         ((uint8_t)DISPLAY_BACKLIGHT_LOWER_VALUE_DEFAULT)
unsigned long displayBacklightUpperValue      = DISPLAY_BACKLIGHT_UPPER_VALUE_DEFAULT;
unsigned long displayBacklightLowerValue      = DISPLAY_BACKLIGHT_LOWER_VALUE_DEFAULT;
uint8_t displayBacklightCurrentValue          = DISPLAY_BACKLIGHT_DEFAULT_VALUE;

#define DISPLAY_ALS_UPPER_THRESHOLD_VALUE_DEFAULT   ((unsigned long) (300))
#define DISPLAY_ALS_LOWER_THRESHOLD_VALUE_DEFAULT   ((unsigned long) (150))
#define DISPLAY_DELAY_BEFORE_CHANGE_VALUE_DEFAULT   ((unsigned long) (10000))  // In mS
unsigned long displayALSUpperThresholdValue       = DISPLAY_ALS_UPPER_THRESHOLD_VALUE_DEFAULT;
unsigned long displayALSLowerThresholdValue       = DISPLAY_ALS_LOWER_THRESHOLD_VALUE_DEFAULT;
unsigned long displayDelayBeforeChangeValue       = DISPLAY_DELAY_BEFORE_CHANGE_VALUE_DEFAULT;
unsigned long displayALSCurrentValue              = 0;
String strAmbientLightLevel("");


typedef enum {
   eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK = 0,
   eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_BRIGHT    = 1,
   eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_DIM       = 2
} eALS_LEVEL_CONTROL_STATE;

eALS_LEVEL_CONTROL_STATE eALS_LEVEL_STATE = eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK;


unsigned long ambient_light_sensor_old = 0;                        // Previous ALS reading
unsigned long ambient_light_sensor_new = 0;                        // New ALS reading
unsigned long previousALSMillis        = 0;                        // previous time value, so routine is non blocking
unsigned long readIntervalALS          = DEFAULT_SENSOR_INTERVAL;  // interval at which to read sensor, in milli seconds (10secs)
boolean sendALSUpdate                  = false;  


// Buttons and Leds
const int lightPin0                 = 7;  // Controls system LED0 
const int lightPin1                 = 6;  // Controls system LED1 
const int buttonPin1                = 3;  // Reads system button 1 input
const int buttonPin0                = 2;  // Reads system button 0 input
const int closeLoggingFilebuttonPin = 1;  // Button to close off logging before power is removed.
const int openLoggingFilebuttonPin  = 0;  // Button to open up logging again.

#define NXP_PCF8574P_ADDRESS 0x3E
#define NXP_PCF8574P_INPUT_MASK 0b00011111 // P7 = O/P System Led0, P6 = O/P System Led1, P5, P4 = I/P , P3 = I/P button 1, P2 = I/P OPEN LOGGING, P1 = I/P CLOSE LOGGING, P0 = I/P button 0
// create an instance of the bounce class
BounceI2C myButton0 = BounceI2C(NXP_PCF8574P_ADDRESS, NXP_PCF8574P_INPUT_MASK);
BounceI2C myButton1 = BounceI2C(NXP_PCF8574P_ADDRESS, NXP_PCF8574P_INPUT_MASK);
boolean isOn0 = false;  // var to toggle button 0 state
boolean isOn1 = false;  // var to read button 1 state
BounceI2C port = BounceI2C(NXP_PCF8574P_ADDRESS, NXP_PCF8574P_INPUT_MASK);

// Logging control defines
//#define DEFAULT_LOGGING_PERIOD_IN_MINUTES 10
#define DEFAULT_LOGGING_PERIOD_IN_MINUTES 1
const char* csvColumnTitles = "Date,Time,TempLocal,HumidityLocal,BarometricPressure,AmbientLightLevel";
int iLoggingPeriodInMinutes = DEFAULT_LOGGING_PERIOD_IN_MINUTES;
int iOldLoggingTimeInMinutes;
int iNewLoggingTimeInMinutes;
int iElapsedLoggingTimeInMinutes;
File logFile;
const char tmpStrSystemFilename[] = "FILENAME.csv";

typedef enum eLoggingStatusTypeTag {
  eLoggingInactive    = 0,
  eLoggingActive      = 1,
  eLoggingInitialise  = 2,
  eLoggingFault       = 3
} eLoggingStatusType;

eLoggingStatusType currentLoggingStatus = eLoggingInactive;
eLoggingStatusType newLoggingStatus = eLoggingInactive;
BounceI2C myCloseLoggingFileButton = BounceI2C(NXP_PCF8574P_ADDRESS, NXP_PCF8574P_INPUT_MASK);
boolean isOncloseLoggingFilebuttonPin = false;
BounceI2C myOpenLoggingFileButton = BounceI2C(NXP_PCF8574P_ADDRESS, NXP_PCF8574P_INPUT_MASK);
boolean isOnopenLoggingFilebuttonPin = false;
//const char fileNameFormat
//const char fileNameFormatString[] = "YYYY-MM-DD_HHMMSS";  // Can't use this as there is a 8.3 file name format restriction on file names for SD card
const char fileNameFormatString[] = "MMDDHHMM";   // MonthDayHoursMinutes
const char fileNameTypeString[] = ".csv";
tmElements_t old_logging_tm;
tmElements_t logging_tm;
char logFilenameOld[sizeof(fileNameFormatString)+ sizeof(fileNameTypeString) + 1];
char logFilename[sizeof(fileNameFormatString)+ sizeof(fileNameTypeString) + 1];
char shortLogFilename[sizeof(fileNameFormatString) + 1];



// For DHT22 Temperature Humidity Sensor
#define DHTPIN 2        // The digital o/p pin we're connected to. GPIO2
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

typedef enum eDHTParameterTypeTag {
  eDHTParameterTemperature  = 0,
  eDHTParameterHumidity     = 1
} eDHTParameterType;

// Values read from Temperature/Humidity sensor
volatile eDHTParameterType whichDHTParameterToRead = eDHTParameterTemperature;

// Values read from Temperature/Humidity sensor
float temp_c_old;                 // Temperature in Centigrade, earlier reading
float temp_c_new;                 // Temperature in Centigrade, latest reading
float humidity_old;               // Relative humidity in %age, earlier reading
float humidity_new;               // Relative humidity in %age, latest reading
float hic_old;                    // Heat Index in Centigrade, earlier reading
float hic_new;                    // Heat Index in Centigrade, latest reading
unsigned long previousMillis = 0; // previous time value, so routine is non blocking
unsigned long readIntervalTempHumi = DEFAULT_TH_SENSOR_INTERVAL;  // interval at which to read Temp/Humi sensor, in milli seconds
float tempCalOffset                = TEMPERATURE_CALIBRATION_OFFSET_DEFAULT;
float tempCalScale                 = TEMPERATURE_CALIBRATION_SCALE_DEFAULT;
float humCalOffset                 = HUMIDITY_CALIBRATION_OFFSET_DEFAULT;
float humCalScale                  = HUMIDITY_CALIBRATION_SCALE_DEFAULT;
String strTempC                    = String(defaultTempStr);
String strHumid                    = String(defaultHumidStr);
String strHIC                      = String(defaultHICStr);


// Temperature and Humidity readings
boolean newTemperatureDataFromWifi = true;
boolean newHumidityDataFromWifi = true;
char tmpTempStr[20+1] = "00.00";
char tmpTempStrOld[20+1] = "00.00";
char tmpHumidStr[20+1] = "00.00";
char tmpHumidStrOld[20+1] = "00.00";
//String tmpHICStr[20+1];
//String tmpHICStrOld[20+1];


int     reportingStrategy = REPORTING_STRATEGY_DEFAULT;
boolean sendTHUpdate      = false;                      // Used to trigger Temp/Humidity update
boolean sendBUpdate       = false;                      // Used to trigger Barometric update
boolean bBrokerPresent    = false;


#define MQTT_VERSION MQTT_VERSION_3_1

#define MQTT_BROKER_IP_STRING_MAX_LEN           30
#define MQTT_BROKER_IP_DEFAULT                  "192.168.1.44"
#define MQTT_BROKER_PORT_DEFAULT                ((int)1883)
#define STA_NETWORK_SSID_DEFAULT                "SENSOR"
#define STA_NETWORK_PASSWORD_DEFAULT            "PASSWORD"
#define AP_NETWORK_SSID_DEFAULT                 "SENSOR"
#define AP_NETWORK_PASSWORD_DEFAULT             "PASSWORD"
#define NETWORK_CONNECTION_ATTEMPTS_DEFAULT     ((int)10)
#define MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT ((int)10)
#define NETWORK_SSID_STRING_MAX_LEN             32  
#define NETWORK_PASSWORD_STRING_MAX_LEN         40
#define CONNECTION_ATTEMPTS_MAX                 100
#define CONNECTION_ATTEMPTS_MIN                 0

char   mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
int    mqtt_broker_port;
String macStrForAPSSID;
char   sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
char   sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
char   ap_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
char   ap_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
int    network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
int    mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
#ifdef DEBUG_GENERAL
int    conDotCountNW   = 0; // Used to print a NW connecting dot each 500ms
int    conDotCountMQTT = 0; // Used to print a MQTT connecting dot each 500ms
#endif



// Topic to publish to, to request this device publish the status of its local software version (Generic Device, MAC Addr, Filename.ino). Caution : This a THSen Broadcast message.
const char* swVerTopic = "WFD/THBSen/SwVer/Command";

char swVerThisDeviceTopic[50];

// Topic to subscribe to, to receive publication of this device's software version. In form (Generic Device, MAC Addr, Filename.ino)
const char* swVerConfirmTopic = "WFD/THBSen/SwVer/Confirm";

// Topic to publish to, to Set this devices Logging period in format 'HH:MM'
const char* setLoggingPeriodCommandTopic = "WFD/THBSen/7/LoggingPeriod/Set/1";

// Topic to publish to, to Get this devices Logging Period Status, returns in format 'HH:MM' via getLoggingPeriodConfirmTopic
const char* getLoggingPeriodStatusTopic = "WFD/THBSen/7/LoggingPeriod/Status/1";

// Topic to subscribe to, to receive Current Logging Period Value of this device
const char* getLoggingPeriodConfirmTopic = "WFD/THBSen/7/LoggingPeriod/Confirm/1";

// Topic to publish to set the Logging status of this device. Open or Close of Logging file, 0 = Close, 1 = Open/Start
const char* setLoggingCommandTopic = "WFD/THBSen/7/Logging/Command/1";

// Topic to publish to, to request the current Logging Status of this device. Response returned via getLoggingStatusConfirmTopic
const char* getLoggingStatusTopic = "WFD/THBSen/7/Logging/Status/1";

// Topic to subscribe to, to recive the Current Logging Status of this device. See 'eLoggingStatusType' for enumeration values
const char* getLoggingStatusConfirmTopic = "WFD/THBSen/7/Logging/Confirm/1";

// Topic to publish to, to set RTC Date of this device. In DD/MM/YY
const char* setDateTopic = "WFD/THBSen/7/Date/Set/1";

// Topic to publish to, to set RTC Time of this device. In HH:MM
const char* setTimeTopic = "WFD/THBSen/7/Time/Set/1";

// Topic to publish to, to set this device's RTC Date and Time with one command. Not implemented
//const char* setDateTimeTopic = "WFD/THBSen/7/Set/DateTime/1";

// Topic to publish to, to control the status of this device's local led state
const char* lightTopic0 = "WFD/THBSen/7/Led/Command/1";

// Topic to publish to, to control the status of this device's local led state
const char* lightTopic1 = "WFD/THBSen/7/Led/Command/2";

// Topic to subscribe to, to receive confirmation that this device has recieved a Led control command
const char* lightConfirmTopic0 = "WFD/THBSen/7/Led/Confirm/1";

// Topic to subscribe to, to receive confirmation that this device has recieved a Led control command
const char* lightConfirmTopic1 = "WFD/THBSen/7/Led/Confirm/2";

// Topic to subscribe to, to receive publication of the status of this devices local button 0
const char* buttonTopic = "WFD/THBSen/7/Button/Command/1";

// Topic to publish to, to request this device send confirmation of the status of the button 0. 
// When publication received will respond with button status via publication on buttonTopic
const char* buttonStatusTopic = "WFD/THBSen/7/Button/Status/1";

// Topic to subscribe to, to receive publication of the status of this devices local button 1
const char* buttonTopic1 = "WFD/THBSen/7/Button/Command/2";

// Topic to publish to, to request this device send confirmation of the status of the button 1. 
// When publication received will respond with button status via publication on buttonTopic
const char* buttonStatusTopic1 = "WFD/THBSen/7/Button/Status/2";

// Topic to subscribe to, to receive publication of the status of this devices local Ambient Light Status
const char* ambientLightSensorTopic = "WFD/THBSen/7/ALSStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local temperature
const char* temperatureTopic = "WFD/THBSen/7/TempStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local humidity
const char* humidityTopic = "WFD/THBSen/7/HumdStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local heat index
const char* heatIndexTopic = "WFD/THBSen/7/HeatIndStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local barometric pressure
const char* barometricTopic = "WFD/THBSen/7/BarometricStatus/1";

// Topic to subscribe to, to request this device publish the status of its local RSSI for SSID
const char* rssiTopic = "WFD/THBSen/7/RSSILev";

// Topic to subscribe to, to receive publication of the status of this devices  local RSSI in dBm
const char* rssiConfirmTopic = "WFD/THBSen/7/RSSILev/Confirm";

// Topic to publish to, to request this device re-format it's local SD filing system. Response; 0 = Done, Error = 1
// Response is sent via 'sdConfirmTopic'
const char* sdInitTopic = "WFD/THBSen/7/SD/Init/Command";

// Topic to publish to, to request this device re-read all the values stored in it's local SD filing system (specifically CALIBRATION_PARAMETERS_FILE). Response; 0 = Done, Error = 1
// Response is sent via 'sdConfirmTopic'
const char* sdReadTopic = "WFD/THBSen/7/SD/Read/Command";

// Topic to publish to, to request this device store a new Temperature Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 3
// Temperature command parameter in units of degrees celcius is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureZeroOffsetTopic = "WFD/THBSen/7/SD/CalVal/Temperature/ZeroOff/Command/1";

// Topic to publish to, to request this device store a new Temperature Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 5
// Temperature command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureScalingFactorTopic = "WFD/THBSen/7/SD/CalVal/Temperature/ScaleFact/Command/1";

// Topic to publish to, to request this device store a new Humidity Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 2
// Humidity command parameter in units of percent is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdHumidityZeroOffsetTopic = "WFD/THBSen/7/SD/CalVal/Humidity/ZeroOff/Command/1";

// Topic to publish to, to request this device store a new Humidity Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 4
// Humidity command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'sdConfirmTopic'
const char* sdHumidityScalingFactorTopic = "WFD/THBSen/7/SD/CalVal/Humidity/ScaleFact/Command/1";

// Topic to publish to, to request this device store a new Barometric Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 6
// Barometric command parameter in units of percent is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdBarometricZeroOffsetTopic = "WFD/THBSen/7/SD/CalVal/Barometric/ZeroOff/Command/1";

// Topic to publish to, to request this device store a new Barometric Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 7
// Barometric command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'sdConfirmTopic'
const char* sdBarometricScalingFactorTopic = "WFD/THBSen/7/SD/CalVal/Barometric/ScaleFact/Command/1";

// Topic to publish to, to request this device to publish the value of a given entry in the CALIBRATION_PARAMETERS_FILE
// SD Send command parameter = 1..n, where n is the value in the CALIBRATION_PARAMETERS_FILE file to return. It effectively 
// represents a given line number and responds in freeform text 
// Response is sent via 'sdConfirmTopic'
// Temperature Zero Offset  = 1
// Temperature Scale Factor = 2
// Humidity Zero Offset     = 3
// Humidity Scale Factor    = 4
// Barometric Zero Offset   = 5
// Barometric Scale Factor  = 6
// Reporting Strategy       = 7
//
const char* sdSendTopic = "WFD/THBSen/7/SD/Send";

// Topic to publish to, to request this device store new Security Values in it's local SD filing system. 
// Responses; 
// 0  = Done, 
// 1  = Failed to open SECURITY_PARAMETERS_FILE for write, 
// 6  = MQTT Broker IP address malformed,
// 7  = MQTT Broker Port number invalid,
// 8  = Network SSID or Network Password Wrong length,
// 9  = MQTT Broker Connection Attempts number invalid,
// 10 = MQTT Broker Connection Attempts out of range,
// 11 = Network Connection Attempts number invalid,
// 12 = Network Connection Attempts out of range,
// 13 = One or more items in the parameter string is missing
// Parameter is in the following form 'BrokerIPAddress,BrokerPort,MQTTBrokerConnectionAttempts,NetworkSSID,NetworkPassword,NetworkConnectionAttempts'
// Where;
// BrokerIPAddress : AAA.BBB.CCC.DDD dotted decimal form
// BrokerPort : Integer form. Typically 1883 for Mosquitto MQTT Broker
// MQTTBrokerConnectionAttempts : CONNECTION_ATTEMPTS_MIN ... CONNECTION_ATTEMPTS_MAX. 0 is a special case meaning keep retrying
// NetworkSSID : Free form text
// NetworkPassword : Free form text
// NetworkConnectionAttempts : Integer form. can be any value as this field is not implemented.
// 
// Response is sent via 'sdConfirmTopic'
const char* sdNewSecValsTopic  = "WFD/THBSen/7/SD/SecVals";

// Topic to subscribe to, to receive publication of response that a given SD command has received and executed 
const char* sdConfirmTopic = "WFD/THBSen/7/SD/Conf";

// Topic to publish to, to request this device control the reporting strategy 
// Reporting Strategy command parameter 0..60, where n is the value in minutes to send a temperature/humidity update. The value 0 means send whenever there is a change. Response; 0 = Done, Error in range = 1
const char* reportingStrategyTopic = "WFD/THBSen/7/RepStrat";

// Topic to publish a pass_response that a given Reporting Strategy command has received and executed
const char* reportingStrategyConfirmTopic = "WFD/THBSen/7/RepStrat/Conf";

// Topic to publish to, to request this device return the content of the system LCD 
// Read System LCD command parameter row 0..3, column 0...19, stringLength 0...19
// Response; Requested String
const char* readSystemLCDTopic = "WFD/THBSen/7/ReadLCD/1";

// Topic to publish the request System LCD content to
const char* readSystemLCDConfirmTopic = "WFD/THBSen/7/ReadLCD/Conf/1";

// Topic to publish to, to request this device display a given configurable sensor entry on the system LCD 
// Display entry number 0 = Local T&H, 0...255 = Sensors
// Response; Display will update with desired configurable sensor entry
const char* displayConfigurableSensorEntryTopic = "WFD/THBSen/7/DisplayCSE/1";

// Topic to publish the request System LCD content to. Response; 0 = Done.
const char* displayConfigurableSensorEntryConfirmTopic = "WFD/THBSen/7/DisplayCSE/Conf/1";


// This line is here to cure the following compiler error;
//
extern  void callback(char* topic, byte* payload, unsigned int length);
//
//  XXXXXXXXX:35: error: 'callback' was not declared in this scope
//
//  PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);
//
// At the time of writing this code it was a known issue with Arduino IDE 1.6.8 and the 2.1.0 esp code this sketch fails to compile:
// TITLE : Function not declared in this scope #1881 
// URL   : https://github.com/esp8266/Arduino/issues/1881
//


const char* subscriptionsArray[] = {swVerThisDeviceTopic, swVerTopic, lightTopic0, lightTopic1,rssiTopic, sdInitTopic, sdReadTopic, sdBarometricZeroOffsetTopic, sdBarometricScalingFactorTopic, sdHumidityZeroOffsetTopic, sdHumidityScalingFactorTopic, sdTemperatureZeroOffsetTopic, sdTemperatureScalingFactorTopic, sdSendTopic, reportingStrategyTopic, sdNewSecValsTopic, buttonStatusTopic, buttonStatusTopic1, setDateTopic, setTimeTopic, setLoggingCommandTopic, getLoggingStatusTopic, setLoggingPeriodCommandTopic, getLoggingPeriodStatusTopic, readSystemLCDTopic, displayConfigurableSensorEntryTopic};
int maxSubscriptions = 0;

#define WiFiConnected (WiFi.status() == WL_CONNECTED)
WiFiClient wifiClient;
PubSubClient MQTTclient(wifiClient);

String clientName;
const char* THIS_GENERIC_DEVICE = "esp8266";
String swVersion;


// Gesture control
#define GESTURE_SENSOR_I2C_ADDRESS 0x3F
APDS9960_NonBlocking gestureSensor(GESTURE_SENSOR_I2C_ADDRESS);
boolean bGestureSensorFail                 = false;  // Flags true at initialisation
boolean bGestureAvailableFlag              = false;
uint8_t uiGestureValue                     = APDS9960_GVAL_NONE;
bool bAllowGestureControl                  = true;   // If set true the system will read the gesture sensor
bool bAllowConfigurableGestureControl      = false;  // If set true the system will allow the use of up and down gestures
#define MAX_MQTT_PAYLOAD_STRING              1024
#define MAX_CONFIGURABLE_GESTURE_INSTANCES   2

typedef struct ConfigurableGestureInstance {
  char   strGestureTopic[MAX_TOPIC_STRING];
  char   strGesturePayload[MAX_MQTT_PAYLOAD_STRING];
} sConfigurableGestureInstance;

sConfigurableGestureInstance ConfigurableGestureInstanceArray[MAX_CONFIGURABLE_GESTURE_INSTANCES];

typedef enum {
   eGESTURE_UP   = 0,
   eGESTURE_DOWN = 1
} eGESTURE;



// Button control
const char strEmptyEntry[] = "null";
bool bAllowConfigurableButtonControl      = false;  // If set true the system will allow the use of configurable buttons
#define MAX_CONFIGURABLE_BUTTON_INSTANCES   3

typedef struct ConfigurableButtonInstance {
  char   strButtonTopic[MAX_TOPIC_STRING];
  char   strButtonPayload[MAX_MQTT_PAYLOAD_STRING];
  bool   bButtonAssigned;
} sConfigurableButtonInstance;

sConfigurableButtonInstance ConfigurableButtonInstanceArray[MAX_CONFIGURABLE_BUTTON_INSTANCES];

typedef enum {
   eBUTTON_0_ON  = 0,
   eBUTTON_0_OFF = 1,
   eBUTTON_1     = 2
} eBUTTON;


// Struct to hold a single timer instance
typedef struct tsTimerInstance {
  void (*tmrcallback)(void);   // Function called when timing period exceeded
  boolean bRunning;            // Flag, set with timer running
  unsigned long ulTimerPeriod; // Timing period in milliseconds
  unsigned long ulStartValue;  // Grab of value from millis() when timer was started, used to calculate elapsed time
} TimerInstance;

#define TOTAL_TIME_IN_MILLISECONDS(H, M, S) ((unsigned long)((((unsigned long)H)*60UL*60UL*1000UL)+(((unsigned long)M)*60UL*1000UL)+(((unsigned long)S)*1000UL)))
#define MAX_TIMERS                      4
#define PERIODIC_UPDATE_TIMER           0
#define LED_FLASH_TIMER                 1
#define DISPLAY_BRIGHTNESS_CHANGE_TIMER 2
#define DISPLAY_SCOLL_UPDATE_TIMER      3


TimerInstance stiTimerArray[MAX_TIMERS];  // Array for holding all the active timer instances


#define FILE_VAR_INSTANCE_TYPE_STRING 0
#define FILE_VAR_INSTANCE_TYPE_FLOAT  1
#define FILE_VAR_INSTANCE_TYPE_INT    2
#define FILE_VAR_INSTANCE_TYPE_BOOL   3

typedef struct tsFileVarInstance {
  int iVarType;  
  void *ptrVar;
} FileVarInstance;


FileVarInstance SecurityVarArray[] = 
{
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)mqtt_broker_ip                  },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&mqtt_broker_port               },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&mqtt_broker_connection_attempts},
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)sta_network_ssid                },
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)sta_network_password            },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&network_connection_attempts    }
};



FileVarInstance CalibrationVarArray[] = 
{
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalOffset     },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalScale      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalOffset      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalScale       },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&baroCalOffset     },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&baroCalScale      },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&reportingStrategy }
};



FileVarInstance ConfigurationVarArray[] = 
{
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&displayALSUpperThresholdValue },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&displayALSLowerThresholdValue },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&displayDelayBeforeChangeValue },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&displayBacklightUpperValue    },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&displayBacklightLowerValue    },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&displayDelayBeforeScrollValue },
  {FILE_VAR_INSTANCE_TYPE_BOOL,  (void *)&bAllowGestureControl          }
};




typedef enum {
   eSENSORSTATE_INIT         = 0,
   eSENSORSTATE_NO_CONFIG    = 1,
   eSENSORSTATE_PENDING_NW   = 2,
   eSENSORSTATE_PENDING_MQTT = 3,
   eSENSORSTATE_ACTIVE       = 4
} eSENSORSTATE;

eSENSORSTATE eSENSORSTATE_STATE = eSENSORSTATE_INIT;

const char* nDNSHostName = "DSKTOPSRV";

ESP8266WebServer server(80);
static bool hasSD = false;
//IPAddress APIPAddress (192,168,1,1);
//IPAddress APNWMask (255,255,255,0);
IPAddress tmpAPIPAddress;


#define LED_FLASH_PERIOD   500 // Time of flash period in mS
#define FLASH_SEQUENCE_MAX 11  // The maximum number of definable states a flash sequence can have

typedef enum {
   eLEDFLASH_NO_CONFIG    = 0,
   eLEDFLASH_PENDING_NW   = 1,
   eLEDFLASH_PENDING_MQTT = 2,
   eLEDFLASH_OFF          = 3,
   eLEDFLASH_SEQUENCE_END = 4
} eLEDFLASHSTATE;

eLEDFLASHSTATE eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
int iFlashSequenceIndex = 0;

char cFlashProfiles[][FLASH_SEQUENCE_MAX] = {
  "1000000000",  // No Config
  "1010000000",  // Pending NW
  "1111111111",  // Pending MQTT
  "0000000000",  // Off
};

#ifdef DEBUG_STATE_CHANGE
char printStateChangeBuf[200];
const char *SensorStates[]= {
  "eSENSORSTATE_INIT",
  "eSENSORSTATE_NO_CONFIG",
  "eSENSORSTATE_PENDING_NW",
  "eSENSORSTATE_PENDING_MQTT",
  "eSENSORSTATE_ACTIVE"
};

char *printStateChange(eSENSORSTATE ThisState, eSENSORSTATE NextState, const char *InThisFunction)
{
    printStateChangeBuf[0] = 0x00;
    sprintf(printStateChangeBuf,"State Change %s => %s : Within '%s'",SensorStates[ThisState],SensorStates[NextState],InThisFunction);
    return printStateChangeBuf;
}

#define SHOW_UPDATED_STATE(t,n,f) Serial.println(printStateChange((t),(n),(f)))
#endif

#ifdef DEBUG_LEDFLASH
char printLedStateChangeBuf[200];
const char *LedStates[]= {
  "eLEDFLASH_NO_CONFIG",
  "eLEDFLASH_PENDING_NW",
  "eLEDFLASH_PENDING_MQTT",
  "eLEDFLASH_OFF",
  "eLEDFLASH_SEQUENCE_END"
};

char *printLedStateChange(eLEDFLASHSTATE ThisState, eLEDFLASHSTATE NextState, const char *InThisFunction)
{
    printLedStateChangeBuf[0] = 0x00;
    sprintf(printLedStateChangeBuf,"LED State Change %s => %s : Within '%s'",LedStates[ThisState],LedStates[NextState],InThisFunction);
    return printLedStateChangeBuf;
}

#define SHOW_UPDATED_LED_STATE(t,n,f) Serial.println(printLedStateChange((t),(n),(f)))
#endif




//##############################################
//###                                        ###
//###          Function Declarations         ###
//###                                        ###
//##############################################
void checkTemperatureAndHumidity(void);
char *readSystemLCD(LiquidCrystal_I2C_PCF8574 lcd, int iRow, int iColumn,int iLength,char *strLCDString);
void grabParm(char **ptrToParmString, String *recipientString);
int fileWrite(File f, FileVarInstance *fviArray, int iTotalParametersToWrite);
int fileRead(File f, FileVarInstance *fviArray, int iTotalParametersToRead);
void readCalibrationValues();
void readNetworkSecurityParameters();
void readConfigurationParameters();
void connectMQTT();
void makeSubscriptions(void);
String macToStr(const uint8_t* mac, boolean addColons);
void timer_create(int iTimerNumber, unsigned long ulTimerPeriod, void (*callbackfn)(void));
void timer_update(void);
void timer_start(int iTimerNumber);
void timer_stop(int iTimerNumber);
void timer_reset(int iTimerNumber);
boolean timer_isRunning(int iTimerNumber);
void timer_change_period(int iTimerNumber, unsigned long ulTimerPeriod);
void ledFlashTimerCallback(void);
void periodicUpdateTimerCallback(void);
void displayBrightnessChangeTimerCallback(void);
void displayScrollUpdateTimerCallback(void);
void returnOK(String mess) ;
void handleNetworkConfig();
void handleNotFound();
boolean isFloat(String tString);
boolean isValidNumber(String str);
bool isValidIpv4Address(char *st);
void checkBarometricPressure(void);
char *fmtDouble(double val, byte precision, char *buf, unsigned bufLen);
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width);
void checkALSValue(void);
void updateAccessPointStatus(boolean state);
void updateWiFiNetworkConnectionStatus(boolean state);
void updateMQTTBrokerConnectionStatus(boolean state);
void loadChars(void);
void updateTimeDate(void);
int extractTime(char *strTime, int *Hours, int *Minutes);
int extractDate(char *strDate, int *Days, int *Months, int *Years);
int parseString(char *delimiter, char *source, ...);
void setBackLightLevel(uint8_t blVal);
sSensorInstance * addItem(sSensorInstance **Head, const char *sSensorName, const char *sTemperatureTopic, const char *sHumidityTopic, const char *sTemp, const char *sHumid, bool bExtSrc);
boolean sensorListEnd(sSensorInstance *Head, sSensorInstance *Tail);
boolean sensorListBegining(sSensorInstance *Head, sSensorInstance *Tail);
sSensorInstance * sensorListGetNext(sSensorInstance **tmpPtr);
sSensorInstance * sensorListGetPrevious(sSensorInstance **tmpPtr);
void subscribeToTopics(sSensorInstance *Head);
void readConfigurableSensorInput(void);
void updateDisplay(void);
void readAPDS9960(boolean *bGestureAvailableFlag, uint8_t *uiGestureValue);
void handleGesture(void);
void readConfigurableUpDownGestureInput(void);
void checkButton(void);
void updateLogging(void);
void readConfigurableButtonInput(void);
void printBarometricTrend(eBAROMETRIC_PRESSURE_STATE CurrentBarometricTrend);
/*
void updateTimeDate(void);
void loadChars(void);
void updateWiFiNetworkConnectionStatus(boolean state);
void updateMQTTBrokerConnectionStatus(boolean state);
void checkBarometricPressure(void);
void updateDisplay(void);
void setBackLightLevel(uint8_t blVal); 
sSensorInstance * addItem(sSensorInstance **Head, const char *sSensorName, const char *sTemperatureTopic, const char *sHumidityTopic, const char *sTemp, const char *sHumid, bool bExtSrc);
boolean sensorListEnd(sSensorInstance *Head, sSensorInstance *Tail);
boolean sensorListBegining(sSensorInstance *Head, sSensorInstance *Tail);
sSensorInstance * sensorListGetNext(sSensorInstance **tmpPtr);
sSensorInstance * sensorListGetPrevious(sSensorInstance **tmpPtr);
*/

void setup() {
  char cTmpRow3[MAX_SYSTEM_LCD_COLUMNS+1];
  // Start the serial line for debugging
  // This is enabled or the TX/RX port will require 10K pull ups to stop oscillations of the I/Ps which makes the ESP8266-01 pull more current and can crash
  //#ifdef DEBUG_GENERAL
  Serial.begin(115200);
  delay(100);
  //#endif

  // Set up I2C
  Wire.begin(I2C_SDA,I2C_SCL);  

  // Initialise system LCD display
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  loadChars();
  lcd.setCursor(0,0);
  lcd.writeStr("NO LOGGING     HH:MM"); // print message
  lcd.setCursor(0,1);
  lcd.writeStr("pppp.ppmB   DD/MM/YY"); // print message
  printBarometricTrend(eSTABLE);
  lcd.setCursor(0,2);
  lcd.writeStr("                    "); // print message 
  lcd.setCursor(0,3);
  sprintf(cTmpRow3,"T:%s C  RH:%s%",defaultTempStr,defaultHumidStr);
  lcd.writeStr(cTmpRow3); // print message 
  //lcd.writeStr("T:tt.tt C  RH:hh.hh%"); // print message 
  lcd.setCursor(7, 3);
  lcd.write(Degrees);
  updateAccessPointStatus(false);
  updateMQTTBrokerConnectionStatus(false);
  updateWiFiNetworkConnectionStatus(false);

  // Initialise Ambient Light Sensor
  displayALSUpperThresholdValue = DISPLAY_ALS_UPPER_THRESHOLD_VALUE_DEFAULT;
  displayALSLowerThresholdValue = DISPLAY_ALS_LOWER_THRESHOLD_VALUE_DEFAULT;
  displayDelayBeforeChangeValue = DISPLAY_DELAY_BEFORE_CHANGE_VALUE_DEFAULT;
  displayALSCurrentValue        = 0;
  // Initalise the LCD system display scrolling var
  displayDelayBeforeScrollValue = DISPLAY_DELAY_BEFORE_SCROLL_VALUE_DEFAULT;
  
  // Initialise backlight control
  displayBacklightUpperValue    = DISPLAY_BACKLIGHT_UPPER_VALUE_DEFAULT;
  displayBacklightLowerValue    = DISPLAY_BACKLIGHT_LOWER_VALUE_DEFAULT;
  displayBacklightCurrentValue  = DISPLAY_BACKLIGHT_DEFAULT_VALUE;
  setBackLightLevel(displayBacklightCurrentValue); 

  strAmbientLightLevel = "";
  eALS_LEVEL_STATE = eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK;

  // Initialise BH1750FVI I2C Ambient Light Intensity Sensor
  sendALSUpdate             = false;
  ambient_light_sensor_old  = 0;                                          
  ambient_light_sensor_new  = 0;                                
  previousALSMillis         = 0;                      
  readIntervalALS           = DEFAULT_SENSOR_INTERVAL;
  
  ambientLightSensor.begin();
  
  // Initialise BMP085 sensor
  bBMPSensorFail = false;
  newBarometricData = false;
  if (!bmp.begin())
  {
      #ifdef DEBUG_GENERAL
      Serial.println("BMP085 sensor not responding");
      #endif
      bBMPSensorFail = true;
      newBarometricData = true;
  }    
  strTempBaro[0]           = 0;                                 
  barometric_old           = 0;                                          
  barometric_new           = 0;                                
  previousBarometricMillis = 0;                      
  baroCalOffset            = BAROMETRIC_CALIBRATION_OFFSET_DEFAULT;
  baroCalScale             = BAROMETRIC_CALIBRATION_SCALE_DEFAULT;
  readIntervalBarometric   = DEFAULT_SENSOR_INTERVAL;

  for (int i = 0; i < MAX_BAROMETRIC_SAMPLES; i++)
    fBarometricRollingAverage[i]   = 0.0;
  bFirstBarometricRead             = true;
  iBarometricRollingAveragePointer = 0;
  iBarometricDeltaCount            = 0;
  fOldBarometricTrendAverage       = 0.0;
  fNewBarometricTrendAverage       = 0.0;
  sendBTrendUpdate                 = false;
  
  // Initialise DHT sensor
  dht.begin();
  humidity_old            = 0.0;
  humidity_new            = 0.0;
  temp_c_old              = 0.0;  
  temp_c_new              = 0.0;  
  hic_old                 = 0.0;     
  hic_new                 = 0.0; 
  humCalOffset            = HUMIDITY_CALIBRATION_OFFSET_DEFAULT;
  humCalScale             = HUMIDITY_CALIBRATION_SCALE_DEFAULT;
  tempCalOffset           = TEMPERATURE_CALIBRATION_OFFSET_DEFAULT;
  tempCalScale            = TEMPERATURE_CALIBRATION_SCALE_DEFAULT;
  reportingStrategy       = REPORTING_STRATEGY_DEFAULT;
  readIntervalTempHumi    = DEFAULT_TH_SENSOR_INTERVAL;
  strTempC                = defaultTempStr;
  strHumid                = defaultHumidStr;
  strHIC                  = defaultHICStr;
  whichDHTParameterToRead = eDHTParameterTemperature;
  
  // Real Time Clock
  RTC.read(tm_old);
  RTC.read(tm); // Dummy read to get a first screen update.
  bNewTimeDateSet = true;
  if (tm_old.Minute == 0)
    tm_old.Minute = 59;
  else
   tm_old.Minute -= 1;
  
  // Initialize the system LEDs as an output and set to Low (off) 
  port.digitalWrite(lightPin0, LOW);  
  port.digitalWrite(lightPin1, LOW);  

  // Generate client name based on MAC address
  clientName = THIS_GENERIC_DEVICE;
  clientName += '-';
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac,true);
  macStrForAPSSID = macToStr(mac,false);
  macStrForAPSSID.trim();
  sprintf(swVerThisDeviceTopic,"WFD/%s/SwVer/Command",macToStr(mac, true).c_str());

  swVersion = THIS_GENERIC_DEVICE;
  swVersion += ',';
  swVersion += macToStr(mac,true);
  swVersion += ',';
  swVersion += __FILENAME__;
  #ifdef DEBUG_GENERAL  
  Serial.print("Client Name : ");
  Serial.println(clientName);
  Serial.print("SW Version : ");
  Serial.println(swVersion);
  #endif

  // Set up default security parameters. If all else fails so this device can become an AP.
  strcpy(ap_network_ssid,AP_NETWORK_SSID_DEFAULT);
  strcat(ap_network_ssid,macStrForAPSSID.c_str());
  strcpy(ap_network_password,AP_NETWORK_PASSWORD_DEFAULT);
  strcpy(mqtt_broker_ip, MQTT_BROKER_IP_DEFAULT);
  mqtt_broker_port = MQTT_BROKER_PORT_DEFAULT;
  mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
  strcpy(sta_network_ssid, STA_NETWORK_SSID_DEFAULT);
  strcpy(sta_network_password, STA_NETWORK_PASSWORD_DEFAULT);
  network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
  bBrokerPresent = true;

  // Set up MQTT auto topic subscription
  maxSubscriptions = sizeof(subscriptionsArray)/sizeof(const char*);  


  // Set initial Button status
  isOn0 = false;  
  isOn1 = false;  

  // Initialise input buttons
  myButton0.attach(buttonPin0);
  myButton0.interval(50);
  myButton1.attach(buttonPin1);
  myButton1.interval(50);

  // Initialise the file open/close buttons
  myCloseLoggingFileButton.attach(closeLoggingFilebuttonPin);
  myCloseLoggingFileButton.interval(50);
  myOpenLoggingFileButton.attach(openLoggingFilebuttonPin);
  myOpenLoggingFileButton.interval(50);
 
  isOncloseLoggingFilebuttonPin = false;
  isOnopenLoggingFilebuttonPin = false;
  iLoggingPeriodInMinutes = DEFAULT_LOGGING_PERIOD_IN_MINUTES;
  if (SD.begin(SS))
  {
    currentLoggingStatus = eLoggingInactive;
    //currentLoggingStatus = eLoggingInactive;
    newLoggingStatus = eLoggingInactive;
    //newLoggingStatus = eLoggingInitialise;
    RTC.read(logging_tm);
    old_logging_tm = logging_tm;
    iOldLoggingTimeInMinutes = CONVERT_TO_MINUTES(old_logging_tm);
    iNewLoggingTimeInMinutes = CONVERT_TO_MINUTES(logging_tm);
    #ifdef DEBUG_GENERAL
    Serial.println("SD initialisation completed ok");
    #endif
    logFilenameOld[0] = 0;
    updateLogging();  
  } else {
    currentLoggingStatus = eLoggingFault;
    newLoggingStatus = eLoggingFault;
    #ifdef DEBUG_GENERAL
    Serial.println("SD initialisation failed! Card not responding");
    #endif
  }  
  delay(2000);
  
  // Try to read the calibration values file. If missing this will set defaults
  readCalibrationValues();
  // Try to read the security paramaters file. If missing this will set the ssid and p/w for the AP
  readNetworkSecurityParameters();

  // Try to read the configuration paramaters file. If missing this will set the defaults for display blanking/brightness
  readConfigurationParameters();

  // Read in configurable Sensor Input
  ptrHeadOfSensorInstances = NULL;
  tmpSensorInstancePtr = NULL;
  readConfigurableSensorInput();
  bScrollDisplay = false;
  bNewTemperatureHumidityData = false;
  // Set up initial conditions
  eSENSORSTATE_STATE = eSENSORSTATE_INIT;
  WiFi.mode(WIFI_OFF);

  // Set up timers
  sendTHUpdate = false;
  sendBUpdate = false;
  sendALSUpdate = false;
  timer_create(PERIODIC_UPDATE_TIMER, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategy, 0), periodicUpdateTimerCallback);   
  if (reportingStrategy>0)
    timer_start(PERIODIC_UPDATE_TIMER);

  iFlashSequenceIndex = 0;
  eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
  timer_create(LED_FLASH_TIMER, LED_FLASH_PERIOD, ledFlashTimerCallback);   
  //timer_create(LED_FLASH_TIMER, TOTAL_TIME_IN_MILLISECONDS(0, LED_FLASH_PERIOD, 0), ledFlashTimerCallback);   
  timer_start(LED_FLASH_TIMER);

  timer_create(DISPLAY_BRIGHTNESS_CHANGE_TIMER, displayDelayBeforeChangeValue, displayBrightnessChangeTimerCallback);   

  timer_create(DISPLAY_SCOLL_UPDATE_TIMER, displayDelayBeforeScrollValue, displayScrollUpdateTimerCallback);   
  if (ptrHeadOfSensorInstances != NULL) {
    timer_start(DISPLAY_SCOLL_UPDATE_TIMER);
    tmpSensorInstancePtr = ptrHeadOfSensorInstances;
  } else
    bOneOff = true;

  // Initialise the APDS9960 Gesture Sensor
  bGestureSensorFail = false;
  if (bAllowGestureControl) {
    bAllowConfigurableGestureControl = false;
    readConfigurableUpDownGestureInput();
    bGestureAvailableFlag = false;
    uiGestureValue = APDS9960_GVAL_NONE;
    if (gestureSensor.init()) { ;
      #ifdef DEBUG_GESTURE
      Serial.println("APDS9960 initialised");
      #endif
    } else { ;
      #ifdef DEBUG_GESTURE
      Serial.println("APDS9960 not responding");
      #endif
      bGestureSensorFail = true;
    }
  }

  // Set up button control
  bAllowConfigurableButtonControl = false;
  readConfigurableButtonInput();
   
  delay(2000);
}




void loop(){
  timer_update();                // Update timers
  checkTemperatureAndHumidity(); // Read Temp and Humidity sensor
  checkBarometricPressure();     // Read Barometric pressure
  checkALSValue();               // Monitor Ambient Light Levels
  updateTimeDate();              // This will update the LCD with the local RTC time
  readAPDS9960(&bGestureAvailableFlag, &uiGestureValue);  // Reads the state of the gesture input flags a triggered state 
  handleGesture();               // Handles any hand gestures
  updateDisplay();               // This will update the LCD with Barometric pressure and T&H values
  checkButton();                 // Read system buttons
  updateLogging();               // This updates the logging control  
  //MDNS.update();                 // Check for any mDNS queries and send responses

  switch (eSENSORSTATE_STATE) {
    case eSENSORSTATE_INIT : //
           WiFi.mode(WIFI_OFF);
           yield();
           //delay(1000);
           if ((SD.exists(SECURITY_PARAMETERS_FILE)) && (bBrokerPresent)) {
              updateAccessPointStatus(true);
              eSENSORSTATE_STATE = eSENSORSTATE_PENDING_NW;
              eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_NW;
              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eSENSORSTATE_INIT,eSENSORSTATE_PENDING_NW,"loop");
              #endif
              WiFi.mode(WIFI_AP_STA);
              delay(1000);
              // Read the security paramaters file. 
              readNetworkSecurityParameters();
              // Start STA wifi subsystem
              WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
              #ifdef DEBUG_GENERAL
              Serial.println("Switching to AP_STA Mode. SecVals Found");
              Serial.print("Connecting to "); Serial.println(sta_network_ssid);
              conDotCountNW = 0;  
              #endif
           } else {
              updateAccessPointStatus(true);
              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eSENSORSTATE_INIT,eSENSORSTATE_NO_CONFIG,"loop");
              #endif
              eSENSORSTATE_STATE = eSENSORSTATE_NO_CONFIG;
              eLEDFLASHSTATE_STATE = eLEDFLASH_NO_CONFIG;
              WiFi.mode(WIFI_AP);
              delay(1000);
              #ifdef DEBUG_GENERAL
              if (bBrokerPresent)
                Serial.println("Switching to AP Mode. No SecVals found");
              else
                Serial.println("Switching to AP Mode. No MQTT Broker found");
              #endif
           }

           // Start AP wifi subsystem
           WiFi.encryptionType(ENC_TYPE_WEP);
           //WiFi.softAPConfig(APIPAddress,APIPAddress,APNWMask);
           WiFi.softAP((const char *)ap_network_ssid, (const char *)ap_network_password);
          
           // Late binding for MQTT client
           MQTTclient.setServer((const char *)mqtt_broker_ip, mqtt_broker_port); 
           MQTTclient.setCallback(callback);
           hasSD = true;

           tmpAPIPAddress = WiFi.softAPIP();
           #ifdef DEBUG_GENERAL
           Serial.print("AP IP address: "); Serial.println(tmpAPIPAddress);
           #endif    
          
           //if (MDNS.begin(nDNSHostName, APIPAddress)) {
           if (MDNS.begin(nDNSHostName)) {
             MDNS.addService("http", "tcp", 80);
             #ifdef DEBUG_MDNS
             Serial.println("MDNS responder started");
             Serial.print("You can now connect to http://");
             Serial.print(nDNSHostName);
             Serial.println(".local");
             #endif
           } else {
             #ifdef DEBUG_MDNS
             Serial.println("MDNS responder failed to start");
             #endif
           }

           // Set up HTTP server
           server.on("/0", HTTP_GET, handleNetworkConfig);
           server.onNotFound(handleNotFound);
           server.begin();
           #ifdef DEBUG_WEB
           Serial.println("HTTP server started");
           #endif
           break;

    case eSENSORSTATE_NO_CONFIG  : // Run only as an access point to allow the user to reconfigure to new network
           server.handleClient();
           yield();
           //delay(10); 
           break;

    case eSENSORSTATE_PENDING_NW : // Run as an access point to allow the user to reconfigure to new network and as a station trying to connnect to NW
           server.handleClient();
           yield();
           //delay(10); 
           if (WiFiConnected) {
              // Start wifi subsystem
              //WiFi.mode(WIFI_STA);  // Switch off access point
              //#ifdef DEBUG_GENERAL
              //Serial.println();
              //Serial.println("Switching to STA Mode. Now WiFi is connected.");
              //#endif
              //WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
              eSENSORSTATE_STATE = eSENSORSTATE_PENDING_MQTT;
              eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_MQTT;
              updateAccessPointStatus(false);
              updateMQTTBrokerConnectionStatus(false);
              updateWiFiNetworkConnectionStatus(true);

              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eSENSORSTATE_PENDING_NW,eSENSORSTATE_PENDING_MQTT,"loop");
              #endif
              
              //print out some more debug once connected
              #ifdef DEBUG_GENERAL
              Serial.println("WiFi connected");  
              Serial.print("IP address: ");
              Serial.println(WiFi.localIP());
              #endif
           } else {
              #ifdef DEBUG_GENERAL
              if (conDotCountNW > 50)
                  conDotCountNW = 0;
              if (conDotCountNW == 0)
                Serial.print(".");
              conDotCountNW++;  
              #endif
           }
           break;
    
    case eSENSORSTATE_PENDING_MQTT : // Try to connect to MQTT Broker
           readCalibrationValues();
           connectMQTT();
           break;
    
    case eSENSORSTATE_ACTIVE : // Run as a WiFi client in active mode
           // Reconnect if connection is lost
           if (!MQTTclient.connected()) {
            #ifdef DEBUG_STATE_CHANGE
            Serial.println();
            Serial.println("Switching to AP_STA Mode. As MQTT has disconnected.");
            #endif
            WiFi.mode(WIFI_AP_STA);
            updateAccessPointStatus(true);
            delay(1000);
            WiFi.encryptionType(ENC_TYPE_WEP);
            //WiFi.softAPConfig(APIPAddress,APIPAddress,APNWMask);
            WiFi.softAP((const char *)ap_network_ssid, (const char *)ap_network_password);
            
            tmpAPIPAddress = WiFi.softAPIP();
            #ifdef DEBUG_GENERAL
            Serial.print("AP IP address: "); Serial.println(tmpAPIPAddress);
            #endif    
            connectMQTT();
           } else //maintain MQTT connection
            MQTTclient.loop();
        
           // Delay to allow ESP8266 WIFI functions to run
           yield();
           //delay(10); 
           break;
  }
}



void checkTemperatureAndHumidity(void)
{
  String s1, s2, s3;
  bool bFreshData = false;

  // Wait at least 10 seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= readIntervalTempHumi) {
    // save the last time you read the sensor 
    previousMillis = currentMillis;   
  
    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
      switch (whichDHTParameterToRead) {
      case eDHTParameterTemperature:
        whichDHTParameterToRead = eDHTParameterHumidity;
        temp_c_new = dht.readTemperature();     // Read temperature as Centigrade
        temp_c_new *= tempCalScale;
        temp_c_new += tempCalOffset;
        #ifdef DEBUG_GENERAL
        Serial.println("Reading Temperature from DHT sensor!");
        #endif

        if (isnan(temp_c_new)) {
          #ifdef DEBUG_GENERAL
          Serial.println("Failed to read Temperature from DHT sensor!");
          #endif
          return;
        }
        break;
      case eDHTParameterHumidity:
        whichDHTParameterToRead = eDHTParameterTemperature;
        humidity_new = dht.readHumidity();          // Read humidity (percent)
        humidity_new *= humCalScale;
        humidity_new += humCalOffset;
        #ifdef DEBUG_GENERAL
        Serial.println("Reading Humidity from DHT sensor!");
        #endif

        if (isnan(humidity_new)) {
          #ifdef DEBUG_GENERAL
          Serial.println("Failed to read Humidity from DHT sensor!");
          #endif
          return;
        }
        break;
    }
    hic_new = dht.computeHeatIndex(temp_c_new, humidity_new, false);   // Compute heat index in Celsius 
    // Check if any reads failed and exit early (to try again).
    bFreshData = true;
  }

  if ((reportingStrategy == 0) and (bFreshData)) {
    bFreshData = false;
    strTempC = String(temp_c_new);
    strHumid = String(humidity_new);  
    strHIC = String(hic_new);
    #ifdef DEBUG_GENERAL
    if ((temp_c_new != temp_c_old)     ||
        (humidity_new != humidity_old) ||
        (hic_new != hic_old)) {
      Serial.println("Rep Strat ==0");
      Serial.print("Humidity: ");
      Serial.print(humidity_new);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(temp_c_new);
      Serial.print(" *C\t");
      Serial.print("Heat index: ");
      Serial.print(hic_new);
      Serial.println(" *C");
    } 
    #endif
    
    if (temp_c_new != temp_c_old)
    {
      s1 = String(temp_c_new);
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
        MQTTclient.publish(temperatureTopic, s1.c_str());  
      if (ptrHeadOfSensorInstances != NULL)  
        strcpy((ptrHeadOfSensorInstances->strTemperature),s1.c_str());
      strTempC = String(temp_c_new);
      temp_c_old = temp_c_new;
      bNewTemperatureHumidityData = true;
    }

    if (humidity_new != humidity_old)
    {
      s1 = String(humidity_new);
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
        MQTTclient.publish(humidityTopic, s1.c_str());        
      if (ptrHeadOfSensorInstances != NULL)  
        strcpy((ptrHeadOfSensorInstances->strHumidity),s1.c_str());
      strHumid = String(humidity_new);  
      humidity_old = humidity_new;
      bNewTemperatureHumidityData = true;
    }

    if (hic_new != hic_old)
    {
      s1 = String(hic_new);
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
        MQTTclient.publish(heatIndexTopic, s1.c_str());        
      strHIC = String(hic_new);
      hic_old = hic_new;
    }

  } else  {
    if (sendTHUpdate == true) {
      sendTHUpdate = false;
      
      if (!isnan(temp_c_new)) {
        strTempC = String(temp_c_new);
        if (temp_c_new != temp_c_old)
        {
          s1 = String(temp_c_new);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(temperatureTopic, s1.c_str());        
          if (ptrHeadOfSensorInstances != NULL)  
            strcpy((ptrHeadOfSensorInstances->strTemperature),s1.c_str());
          temp_c_old = temp_c_new;
          bNewTemperatureHumidityData = true;
        }
      } else {
        s1 = String(temp_c_old);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(temperatureTopic, s1.c_str());        
        if (ptrHeadOfSensorInstances != NULL)  
          strcpy((ptrHeadOfSensorInstances->strTemperature),s1.c_str());
        bNewTemperatureHumidityData = true;
      }
      
      if (!isnan(humidity_new)) {
        strHumid = String(humidity_new); 
        if (humidity_new != humidity_old)
        {
          s1 = String(humidity_new);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(humidityTopic, s1.c_str());        
          if (ptrHeadOfSensorInstances != NULL)  
            strcpy((ptrHeadOfSensorInstances->strHumidity),s1.c_str());
          humidity_old = humidity_new;
          bNewTemperatureHumidityData = true;
        }
      } else {
        s1 = String(humidity_old);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(humidityTopic, s1.c_str());        
        if (ptrHeadOfSensorInstances != NULL)  
          strcpy((ptrHeadOfSensorInstances->strHumidity),s1.c_str());
        bNewTemperatureHumidityData = true;
      }
      
      if ((!isnan(humidity_new)) && (!isnan(temp_c_new))) {
        hic_new = dht.computeHeatIndex(temp_c_new, humidity_new, false);   // Compute heat index in Celsius 
        strHIC = String(hic_new);
        if (hic_new != hic_old)
        {
          s1 = String(hic_new);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(heatIndexTopic, s1.c_str());        
          hic_old = hic_new;
        }
      } else {
          s1 = String(hic_old);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(heatIndexTopic, s1.c_str());        
      }
      #ifdef DEBUG_GENERAL
      Serial.println("Rep Strat <> 0");
      Serial.print("Humidity: ");
      Serial.print(humidity_new);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(temp_c_new);
      Serial.print(" *C\t");
      Serial.print("Heat index: ");
      Serial.print(hic_new);
      Serial.println(" *C");
      #endif
    }
  }
}



void callback(char* topic, byte* payload, unsigned int length) {
  int Hour, Min;
  int Day, Month, Year;
  sSensorInstance *tmpHeadPtr = ptrHeadOfSensorInstances;

  //convert topic to string to make it easier to work with
  String topicStr = topic; 
  char tmpCharBuf[length+1];
  int tmpInt = 0;

  for (int i=0;i<length;i++) 
    tmpCharBuf[i] = ((char)payload[i]);
  tmpCharBuf[length] = 0x00;
  
  //Print out some debugging info
  #ifdef DEBUG_GENERAL  
  Serial.println("Callback update.");
  Serial.print("Topic: ");
  Serial.println(topicStr);
  Serial.print("Payload: ");
  Serial.println(tmpCharBuf);
  #endif

  
  //turn light0 off if the payload is '0' and publish to the MQTT server a confirmation message
  if (strcmp(lightTopic0,topic)== 0) {
    if(payload[0] == '1'){ //turn the light on if the payload is '1' and publish the confirmation 
      port.digitalWrite(lightPin0, HIGH);
      MQTTclient.publish(lightConfirmTopic0, "On");
    } else if (payload[0] == '0'){ //turn the light off if the payload is '0' and publish the confirmation
      port.digitalWrite(lightPin0, LOW);
      MQTTclient.publish(lightConfirmTopic0, "Off");
    } else {
      MQTTclient.publish(lightConfirmTopic0, "Err");
    }
    return;
  }

  //turn light1 off if the payload is '0' and publish to the MQTT server a confirmation message
  if (strcmp(lightTopic1,topic)== 0) {
    if(payload[0] == '1'){ //turn the light on if the payload is '1' and publish the confirmation 
      port.digitalWrite(lightPin1, HIGH);
      MQTTclient.publish(lightConfirmTopic1, "On");
    } else if (payload[0] == '0'){ //turn the light off if the payload is '0' and publish the confirmation
      port.digitalWrite(lightPin1, LOW);
      MQTTclient.publish(lightConfirmTopic1, "Off");
    } else {
      MQTTclient.publish(lightConfirmTopic1, "Err");
    }
    return;
  }

  if (strcmp(buttonStatusTopic,topic)== 0) {
    if(isOn0 == false){
      MQTTclient.publish(buttonTopic, "Released");
    } else {
      MQTTclient.publish(buttonTopic, "Pressed");
    }
    return;
  }  

  if (strcmp(buttonStatusTopic1,topic)== 0) {
    if(isOn1 == false){
      MQTTclient.publish(buttonTopic1, "Released");
    } else {
      MQTTclient.publish(buttonTopic1, "Pressed");
    }
    return;
  }  


  if (strcmp(swVerTopic,topic)== 0) {
    MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());
    return;
  }  

  if (strcmp(swVerThisDeviceTopic,topic)== 0) {
    MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());
    return;
  }  

  // handle RSSI topic, send MQTT confirmation via rssiConfirmTopic  
  if (strcmp(rssiTopic,topic)== 0) {
    int32_t rssi = WiFi.RSSI();
    sprintf(tmpCharBuf,"%ld",rssi);
    MQTTclient.publish(rssiConfirmTopic, tmpCharBuf);
    return;
  }    

/*
  // SD Handlers
  // Re-initialise the filing system
  if (strcmp(sdInitTopic,topic)== 0) {
    if (SD.format())
      MQTTclient.publish(sdConfirmTopic, "0");
    else
      MQTTclient.publish(sdConfirmTopic, "1");
    return;
  }  
*/
  // Re-read all stored calibration values
  if (strcmp(sdReadTopic,topic)== 0) {
    String s;
    // open file for readting
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      fileRead(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  
  

  // Write Humidity calibration value to cal file and update local Humidity Cal offset
  if (strcmp(sdHumidityZeroOffsetTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpHumidityCalVal = tmpCharBuf;
    if (!isFloat(tmpHumidityCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "2");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      humCalOffset = tmpHumidityCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  

  // Write Humidity calibration value to cal file and update local Humidity Cal scaling factor
  if (strcmp(sdHumidityScalingFactorTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpHumidityCalVal = tmpCharBuf;
    if (!isFloat(tmpHumidityCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "4");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      humCalScale = tmpHumidityCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  
    
  // Write Barometric calibration value to cal file and update local Barometric Cal offset
  if (strcmp(sdBarometricZeroOffsetTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpBarometricCalVal = tmpCharBuf;
    if (!isFloat(tmpBarometricCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "6");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      baroCalOffset = tmpBarometricCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  
  
  // Write Barometric calibration value to cal file and update local Barometric Cal scaling factor
  if (strcmp(sdBarometricScalingFactorTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpBarometricCalVal = tmpCharBuf;
    if (!isFloat(tmpBarometricCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "7");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      baroCalScale = tmpBarometricCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  


  // Write Temperature calibration value to cal file and update local Temperature Cal offset
  if (strcmp(sdTemperatureZeroOffsetTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpTemperatureCalVal = tmpCharBuf;
    if (!isFloat(tmpTemperatureCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "3");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      tempCalOffset = tmpTemperatureCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  

  // Write Temperature calibration value to cal file and update local Temperature Cal scaling factor
  if (strcmp(sdTemperatureScalingFactorTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpTemperatureCalVal = tmpCharBuf;
    if (!isFloat(tmpTemperatureCalVal))
    {
      MQTTclient.publish(sdConfirmTopic, "5");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "1");
      return;
    } else {
      tempCalScale = tmpTemperatureCalVal.toFloat();
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      f.close();
      MQTTclient.publish(sdConfirmTopic, "0");
      return;
    }
  }  

  // Write Reporting Strategy value to cal file and update local Reporting Strategy variable
  if (strcmp(reportingStrategyTopic,topic)== 0) {
    // test to see value is a float
    String s;
    String tmpReportingStrategyVal = tmpCharBuf;
    #ifdef DEBUG_SD
    Serial.println(tmpReportingStrategyVal);
    #endif
    int tmpVal = tmpReportingStrategyVal.toInt();
    if ((tmpVal < LOWER_REPORTING_STRATEGY_VALUE) || (tmpVal > UPPER_REPORTING_STRATEGY_VALUE))
    {
      MQTTclient.publish(reportingStrategyConfirmTopic, "2");
      return;
    }
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
    if (!f) {
      MQTTclient.publish(reportingStrategyConfirmTopic, "1");
      return;
    } else {
      reportingStrategy=tmpVal;
      fileWrite(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
      if (reportingStrategy == 0)
        timer_stop(PERIODIC_UPDATE_TIMER);
      else {
        timer_change_period(PERIODIC_UPDATE_TIMER, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategy, 0));  
        timer_start(PERIODIC_UPDATE_TIMER);                      
      }
      sendTHUpdate = false;
      sendBUpdate = false;
      f.close();
      MQTTclient.publish(reportingStrategyConfirmTopic, "0");
      return;
    }
  }

  // Query a stored calibration value and publish this value
  if (strcmp(sdSendTopic,topic)== 0) {
    String s;
    // open file for writing
    File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
    if (!f) {
      MQTTclient.publish(sdConfirmTopic, "Err no file");
      return;
    } else {
      //int x = os_sscanf(tmpCharBuf, "%d", &tmpInt);
      tmpInt = String(tmpCharBuf).toInt();
      int iMaxCalParms = (int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance));
      for (int i = 0; i <= iMaxCalParms; i++){
        s=f.readStringUntil('\n');
        s.trim();
        if (i == (tmpInt-1)) {
          MQTTclient.publish(sdConfirmTopic, s.c_str());
          return;
        }
        if (f.position()>= f.size()) break;
      }
      f.close();
      MQTTclient.publish(sdConfirmTopic, "Err Parm Not Found");
      return;
    }
  }   

  // Write new network security values to file and restart IoT device
  if (strcmp(sdNewSecValsTopic,topic)== 0) {
    char  *StrPtr = tmpCharBuf;
    char   tmp_mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
    int    tmp_mqtt_broker_port;
    int    tmp_mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
    char   tmp_sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
    char   tmp_sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
    int    tmp_network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
    String strMQTTBrokerIPAddress;
    String strMQTTBrokerPort;
    String strMQTTBrokerConnectionAttempts;
    String strNetworkSSID;
    String strNetworkPassword;
    String strNetworkConnectionAttempts;

    grabParm(&StrPtr,&strMQTTBrokerIPAddress);
    grabParm(&StrPtr,&strMQTTBrokerPort);
    grabParm(&StrPtr,&strMQTTBrokerConnectionAttempts);
    grabParm(&StrPtr,&strNetworkSSID);
    grabParm(&StrPtr,&strNetworkPassword);
    grabParm(&StrPtr,&strNetworkConnectionAttempts);

/*    
    //sscanf(tmpCharBuf,"%s,%d,%d,%s,%s,%d",tmp_mqtt_broker_ip,&tmp_mqtt_broker_port,&tmp_mqtt_broker_connection_attempts,tmp_sta_network_ssid,tmp_sta_network_password,&tmp_network_connection_attempts);
    os_sprintf(tmpCharBuf,"%s,%d,%d,%s,%s,%d",tmp_mqtt_broker_ip,&tmp_mqtt_broker_port,&tmp_mqtt_broker_connection_attempts,tmp_sta_network_ssid,tmp_sta_network_password,&tmp_network_connection_attempts);
    strMQTTBrokerIPAddress = tmp_mqtt_broker_ip;
    strMQTTBrokerPort = tmp_mqtt_broker_port;
    strMQTTBrokerConnectionAttempts = tmp_mqtt_broker_connection_attempts;
    strNetworkSSID = tmp_sta_network_ssid;
    strNetworkPassword = tmp_sta_network_password;
    strNetworkConnectionAttempts = tmp_network_connection_attempts;
*/    
    #ifdef DEBUG_SECVALS
    Serial.print("SecValsMQTTBrokerIPAddress : "); Serial.println(strMQTTBrokerIPAddress);
    Serial.print("SecValsMQTTBrokerPort : "); Serial.println(strMQTTBrokerPort);
    Serial.print("SecValsMQTTBrokerConnectionAttempts : "); Serial.println(strMQTTBrokerConnectionAttempts);
    Serial.print("SecValsSTANetworkSSID : "); Serial.println(strNetworkSSID);
    Serial.print("SecValsSTANetworkPassword : "); Serial.println(strNetworkPassword);
    Serial.print("SecValsNetworkConnectionAttempts : "); Serial.println(strNetworkConnectionAttempts);
    #endif

    strMQTTBrokerIPAddress.trim();
    strMQTTBrokerPort.trim();
    strMQTTBrokerConnectionAttempts.trim();
    strNetworkSSID.trim();
    strNetworkPassword.trim();
    strNetworkConnectionAttempts.trim();

    if ((strMQTTBrokerIPAddress.length()          == 0) || 
        (strMQTTBrokerPort.length()               == 0) || 
        (strMQTTBrokerConnectionAttempts.length() == 0) || 
        (strNetworkSSID.length()                  == 0) || 
        (strNetworkPassword.length()              == 0) || 
        (strNetworkConnectionAttempts.length()    == 0)) {
      MQTTclient.publish(sdConfirmTopic, "13");
      return;
    }
    
    strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
    if (! isValidIpv4Address((char *)strMQTTBrokerIPAddress.c_str())) {
        MQTTclient.publish(sdConfirmTopic, "6");
        return;
    } else {
      //strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
      if (! isValidNumber(strMQTTBrokerPort)) {
        MQTTclient.publish(sdConfirmTopic, "7");
        return;
      } else {
        tmp_mqtt_broker_port = strMQTTBrokerPort.toInt();
        if (((strNetworkSSID.length() == 0)     || (strNetworkSSID.length() >= NETWORK_SSID_STRING_MAX_LEN)) || 
            ((strNetworkPassword.length() == 0) || (strNetworkPassword.length() >= NETWORK_PASSWORD_STRING_MAX_LEN))) {
            MQTTclient.publish(sdConfirmTopic, "8");
            return;
        } else {
          strcpy(tmp_sta_network_ssid,strNetworkSSID.c_str());
          strcpy(tmp_sta_network_password,strNetworkPassword.c_str());
  
          if (! isValidNumber(strMQTTBrokerConnectionAttempts)) {
            MQTTclient.publish(sdConfirmTopic, "9");
            return;
          } else {
            tmp_mqtt_broker_connection_attempts = strMQTTBrokerConnectionAttempts.toInt();
            if ((tmp_mqtt_broker_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_mqtt_broker_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
              MQTTclient.publish(sdConfirmTopic, "10");
              return;
            } else {
              if (! isValidNumber(strNetworkConnectionAttempts)) {
                MQTTclient.publish(sdConfirmTopic, "11");
                return;
              } else {
                tmp_network_connection_attempts = strNetworkConnectionAttempts.toInt();
                if ((tmp_network_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_network_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
                  MQTTclient.publish(sdConfirmTopic, "12");
                  return;
                } else {
                  strcpy(mqtt_broker_ip,tmp_mqtt_broker_ip);
                  mqtt_broker_port = tmp_mqtt_broker_port;
                  mqtt_broker_connection_attempts = tmp_mqtt_broker_connection_attempts;
                  strcpy(sta_network_ssid,tmp_sta_network_ssid);
                  strcpy(sta_network_password,tmp_sta_network_password);
                  network_connection_attempts = tmp_network_connection_attempts;
                  // Save new network parameters
                  File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
                  if (!f) {
                    MQTTclient.publish(sdConfirmTopic, "1");
                    return;
                  } else {
                    fileWrite(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
                    f.close();
                    MQTTclient.publish(sdConfirmTopic, "0");
                    bBrokerPresent = true;
                    #ifdef DEBUG_STATE_CHANGE
                    SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"callback, sdNewSecValsTopic");
                    #endif
                    eSENSORSTATE_STATE = eSENSORSTATE_INIT;
                    #ifdef DEBUG_SECVALS
                    Serial.print("SecValsMQTTBrokerIPAddress : "); Serial.println(mqtt_broker_ip);
                    Serial.print("SecValsMQTTBrokerPort : "); Serial.println(mqtt_broker_port);
                    Serial.print("SecValsMQTTBrokerConnectionAttempts : "); Serial.println(mqtt_broker_connection_attempts);
                    Serial.print("SecValsSTANetworkSSID : "); Serial.println(sta_network_ssid);
                    Serial.print("SecValsSTANetworkPassword : "); Serial.println(sta_network_password);
                    Serial.print("SecValsNetworkConnectionAttempts : "); Serial.println(network_connection_attempts);
                    #endif
                    return;
                  }      
                }
              }
            }
          }
        }
      }
    }
  }  


  if (strcmp(setDateTopic,topic)== 0) {
    if (length != (sizeof(strDateFormat)-1)) return; 
    if (extractDate(tmpCharBuf, &Day, &Month, &Year) != 3) return;  
//    if (sscanf(tmpCharBuf, "%d/%d/%d", &Day, &Month, &Year) != 3) return;
    #ifdef DEBUG_DT
    Serial.println("Setting Date");
    #endif
    tm.Day = Day;
    tm.Month = Month;
    Year += 2000;
    tm.Year = CalendarYrToTm(Year);
    RTC.write(tm);
    bNewTimeDateSet = true;
    return;
  }
 
  if (strcmp(setTimeTopic,topic)== 0) {
    if (length != (sizeof(strTimeFormat)-1)) return; 
    if (extractTime(tmpCharBuf, &Hour, &Min) != 2) return;
//    if (sscanf(tmpCharBuf, "%d:%d", &Hour, &Min) != 2) return;
    #ifdef DEBUG_DT
    Serial.println("Setting Time");
    #endif

    tm.Hour = Hour;
    tm.Minute = Min;
    tm.Second = 0;
    RTC.write(tm);
    bNewTimeDateSet = true;
    return;
  }
  
  //setDateTimeTopic  Not implemented

  if (strcmp(setLoggingCommandTopic,topic)== 0) {
    if (length == 1) {
      switch (payload[0])
      {
        case  '0' : newLoggingStatus = eLoggingInactive;
                    break;
        case  '1' : newLoggingStatus = eLoggingInitialise;
                    break;
        default : ;
      } 
    }
    return;
  }  

  if (strcmp(getLoggingStatusTopic,topic)== 0) {
    tmpCharBuf[0] = (char)(currentLoggingStatus + '0');
    tmpCharBuf[1] = 0;
    MQTTclient.publish(getLoggingStatusConfirmTopic, tmpCharBuf);
    return;
  }  

  if (strcmp(setLoggingPeriodCommandTopic,topic)== 0) {
    if (length != (sizeof(strTimeFormat)-1)) return; 
    if (extractTime(tmpCharBuf, &Hour, &Min) != 2) return;
//    if (sscanf(tmpCharBuf, "%d:%d", &Hour, &Min) != 2) return;

    if (TOTAL_TIME_IN_MINUTES(Hour, Min) > TWENTYFOUR_HOURS_IN_MINUTES)
      iLoggingPeriodInMinutes = TWENTYFOUR_HOURS_IN_MINUTES;
    else {
      if (TOTAL_TIME_IN_MINUTES(Hour, Min) == 0)
        iLoggingPeriodInMinutes = DEFAULT_LOGGING_PERIOD_IN_MINUTES;
      else
        iLoggingPeriodInMinutes = TOTAL_TIME_IN_MINUTES(Hour, Min);
    }
    #ifdef DEBUG_LOGGING
    Serial.print("Setting Logging Period : ");
    Serial.println(iLoggingPeriodInMinutes);
    #endif

    RTC.read(logging_tm);
    old_logging_tm = logging_tm;      
    return;
  }

  if (strcmp(getLoggingPeriodStatusTopic,topic)== 0) {
    sprintf(tmpCharBuf,"%02d:%02d",(int)(iLoggingPeriodInMinutes/60),(int)(iLoggingPeriodInMinutes-((iLoggingPeriodInMinutes/60)*60)));
    MQTTclient.publish(getLoggingPeriodConfirmTopic, tmpCharBuf);
    return;
  }  

  if (strcmp(readSystemLCDTopic,topic)== 0) {
    char strRow[10], strColumn[10], strLength[10], strLCDString[42];
    int iRow, iColumn, iLength;
    strRow[0]    = '\0';
    strColumn[0] = '\0';
    strLength[0] = '\0';
    int iParmCount = parseString((char *)",", tmpCharBuf, strRow, strColumn, strLength);
    #ifdef DEBUG_PARSER
    {
      char tmpStr[200];
      sprintf(tmpStr,"readSystemLCDTopic : %s,%s,%s, PCount %d",strRow,strColumn,strLength,iParmCount);
      Serial.println(tmpStr);
    }
    #endif

    if ((iParmCount != 3) || (strlen(strRow)==0) || (strlen(strColumn)==0) || (strlen(strLength)==0)) return;
    
    if (iParmCount > 0) {
      String R(strRow), C(strColumn), L(strLength);
      R.trim();
      C.trim();
      L.trim();
      #ifdef DEBUG_PARSER
      {
        char tmpStr[200];
        sprintf(tmpStr,"readSystemLCDTopic : %s,%s,%s, PCount %d",R.c_str(),C.c_str(),L.c_str(),iParmCount);
        Serial.println(tmpStr);
      }
      #endif
      iRow    = R.toInt();
      iColumn = C.toInt();
      iLength = L.toInt();
      if ((iRow > 3) || (iColumn > 19) || (iLength > 19)) return;  
      strLCDString[0] = '\0';
      if (readSystemLCD(lcd, iRow,iColumn,iLength,strLCDString) != '\0') {
        MQTTclient.publish(readSystemLCDConfirmTopic, strLCDString);  // Return LCD string
        return;  
      }
    }
    return;
  }

  if (strcmp(displayConfigurableSensorEntryTopic,topic)== 0) {
    sSensorInstance *tmpHeadPtr = ptrHeadOfSensorInstances;
    String whichConfigurableSensorEntry(tmpCharBuf);
    int iMaxCSE = whichConfigurableSensorEntry.toInt();
    for (int iIndex = 0; iIndex < iMaxCSE; iIndex++)
      sensorListGetNext(&tmpHeadPtr);
    bScrollDisplay = true;
    tmpSensorInstancePtr = tmpHeadPtr;
    timer_start(DISPLAY_SCOLL_UPDATE_TIMER);
    MQTTclient.publish(displayConfigurableSensorEntryConfirmTopic, "0");
    return;
  }  

  
  // Grabs the latest temperature and humidity values and updates the linked list
  if (tmpHeadPtr != NULL) {
    do {
      if (strcmp((const char *) tmpHeadPtr->strTemperatureTopic,topic)== 0) {
        if (strlen(defaultTempStr) == strlen(tmpCharBuf))
          strcpy((char *) tmpHeadPtr->strTemperature, (char *) tmpCharBuf);
        return;
      } else {
        if (strcmp((const char *) tmpHeadPtr->strHumidityTopic,topic)== 0) {
          if (strlen(defaultHumidStr) == strlen(tmpCharBuf))
            strcpy((char *) tmpHeadPtr->strHumidity, (char *) tmpCharBuf);
          return;
        } 
      }
      sensorListGetNext(&tmpHeadPtr);
    } while (!sensorListBegining(ptrHeadOfSensorInstances,tmpHeadPtr));  
  }
}


char *readSystemLCD(LiquidCrystal_I2C_PCF8574 lcd, int iRow, int iColumn,int iLength,char *strLCDString){
  uint8_t ui8RowOffset[] = {0x00, 0x40, 0x14, 0x54};
  for (uint8_t ui8Index = 0; ui8Index < iLength; ui8Index++)
  {
    strLCDString[ui8Index]     = (char) lcd.readDDRam(ui8RowOffset[iRow] + (uint8_t)iColumn + ui8Index);
    strLCDString[ui8Index + 1] = '\0';
  }
  return strLCDString;
}



void grabParm(char **ptrToParmString, String *recipientString){
  #ifdef DEBUG_PARMGRAB
  Serial.print("**ptrToParmString : "); 
  #endif
  while (**ptrToParmString)
  {
    #ifdef DEBUG_PARMGRAB
    Serial.print(**ptrToParmString);
    #endif
    *recipientString += **ptrToParmString;
    (*ptrToParmString)++;
    if ((**ptrToParmString=='\0') || (**ptrToParmString==','))
    {
      if (**ptrToParmString==',')
        (*ptrToParmString)++;
      #ifdef DEBUG_PARMGRAB
      Serial.println();
      #endif
      return;
    }
  }
}


int fileWrite(File f, FileVarInstance *fviArray, int iTotalParametersToWrite) {
    String s;
    for (int i = 0; i < iTotalParametersToWrite; i++){
      switch (fviArray[i].iVarType){
        case FILE_VAR_INSTANCE_TYPE_STRING :
                f.println((char *)(fviArray[i].ptrVar));
                break;
        case FILE_VAR_INSTANCE_TYPE_FLOAT :
                char tmpStr[10];
                dtostrf(*((float *)(fviArray[i].ptrVar)),5,2,tmpStr); // dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
                s = tmpStr;
                s.trim();
                f.println(s.c_str());
                //f.println(tmpStr);
                break;
        case FILE_VAR_INSTANCE_TYPE_INT :
                f.println(*((int *)(fviArray[i].ptrVar)));
                break;
        case FILE_VAR_INSTANCE_TYPE_BOOL :
                f.println( ((*((int *)(fviArray[i].ptrVar)))?"1":"0") );
                break;
        default :
                return 1;
      }
  }
  return 0;
}



int fileRead(File f, FileVarInstance *fviArray, int iTotalParametersToRead) {
    String s;
    for (int i = 0; i < iTotalParametersToRead; i++){
      s=f.readStringUntil('\n');
      s.trim();
      switch (fviArray[i].iVarType){
        case FILE_VAR_INSTANCE_TYPE_STRING :
                strcpy((char *)(fviArray[i].ptrVar),s.c_str());
                break;
        case FILE_VAR_INSTANCE_TYPE_FLOAT :
                *((float *)(fviArray[i].ptrVar)) = s.toFloat();
                break;
        case FILE_VAR_INSTANCE_TYPE_INT :
                *((int *)(fviArray[i].ptrVar)) = s.toInt();
                break;
        case FILE_VAR_INSTANCE_TYPE_BOOL :
                *((int *)(fviArray[i].ptrVar)) = (s.toInt()==0?false:true);
                break;
        default : // Unknown data type
                return 1;
      }
  }
  return 0; // Successful completion
}




void readCalibrationValues()
{
  // open file for reading
  String s;
  File f = SD.open(CALIBRATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
  if (!f) {
    tempCalOffset = TEMPERATURE_CALIBRATION_OFFSET_DEFAULT;
    tempCalScale  = TEMPERATURE_CALIBRATION_SCALE_DEFAULT;
    humCalOffset  = HUMIDITY_CALIBRATION_OFFSET_DEFAULT;
    humCalScale   = HUMIDITY_CALIBRATION_SCALE_DEFAULT;
    baroCalOffset = BAROMETRIC_CALIBRATION_OFFSET_DEFAULT;
    baroCalScale  = BAROMETRIC_CALIBRATION_SCALE_DEFAULT;
    reportingStrategy = REPORTING_STRATEGY_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Cal Vals");
    #endif
  } else {
    fileRead(f, CalibrationVarArray,(int)(sizeof(CalibrationVarArray)/sizeof(tsFileVarInstance)));
    f.close();
  }
  #ifdef DEBUG_GENERAL
  Serial.println("readCalibrationValues");
  s=tempCalOffset;
  Serial.print("Temp Cal Offset : ");    Serial.println(s);
  s=tempCalScale;
  Serial.print("Temp Cal Scale : ");    Serial.println(s);
  s=humCalOffset;
  Serial.print("Humi Cal Offset : ");    Serial.println(s);
  s=humCalScale;
  Serial.print("Humi Cal Scale : ");    Serial.println(s);
  s=baroCalOffset;
  Serial.print("Baro Cal Offset : ");    Serial.println(s);
  s=baroCalScale;
  Serial.print("Baro Cal Scale : ");    Serial.println(s);
  s=reportingStrategy;
  Serial.print("Reporting Strategy : "); Serial.println(s);
  #endif
}



void readNetworkSecurityParameters(){
  // open file for reading
  String s;
  File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_READ_MODE);
  if (!f) {
    strcpy(mqtt_broker_ip, MQTT_BROKER_IP_DEFAULT);
    mqtt_broker_port = MQTT_BROKER_PORT_DEFAULT;
    mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
    strcpy(sta_network_ssid, STA_NETWORK_SSID_DEFAULT);
    strcpy(sta_network_password, STA_NETWORK_PASSWORD_DEFAULT);
    network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Sec Vals. Using defaults");
    #endif
  } else {
    fileRead(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
    f.close();
  }
  strcpy(ap_network_ssid,AP_NETWORK_SSID_DEFAULT);
  strcat(ap_network_ssid,macStrForAPSSID.c_str());
  strcpy(ap_network_password,AP_NETWORK_PASSWORD_DEFAULT);
  #ifdef DEBUG_GENERAL
  Serial.println("readNetworkSecurityParameters");
  Serial.print("Broker IP : ");            Serial.println(mqtt_broker_ip);
  Serial.print("Broker Port : ");          Serial.println(mqtt_broker_port);
  Serial.print("Max MQTT Conn Atmpts : "); Serial.println(mqtt_broker_connection_attempts);
  Serial.print("STA SSID : ");             Serial.println(sta_network_ssid);
  Serial.print("STA PW : ");               Serial.println(sta_network_password);
  Serial.print("Max NW Conn Atmpts : ");   Serial.println(network_connection_attempts);
  Serial.print("AP SSID : ");              Serial.println(ap_network_ssid);
  Serial.print("AP PW : ");                Serial.println(ap_network_password);
  #endif
}




void readConfigurationParameters()
{
  // open file for reading
  String s;
  displayALSCurrentValue        = 0;
  displayBacklightCurrentValue  = DISPLAY_BACKLIGHT_DEFAULT_VALUE;
  File f = SD.open(CONFIGURATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
  if (!f) {
    displayALSUpperThresholdValue = DISPLAY_ALS_UPPER_THRESHOLD_VALUE_DEFAULT;
    displayALSLowerThresholdValue = DISPLAY_ALS_LOWER_THRESHOLD_VALUE_DEFAULT;
    displayDelayBeforeChangeValue = DISPLAY_DELAY_BEFORE_CHANGE_VALUE_DEFAULT;

    displayBacklightUpperValue    = DISPLAY_BACKLIGHT_UPPER_VALUE_DEFAULT;
    displayBacklightLowerValue    = DISPLAY_BACKLIGHT_LOWER_VALUE_DEFAULT;
    
    displayDelayBeforeScrollValue = DISPLAY_DELAY_BEFORE_SCROLL_VALUE_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Conf Vals");
    #endif
  } else {
    fileRead(f, ConfigurationVarArray,(int)(sizeof(ConfigurationVarArray)/sizeof(tsFileVarInstance)));
    f.close();
  }
  #ifdef DEBUG_GENERAL
  Serial.println("readConfigurationValues");
  s=displayALSUpperThresholdValue;
  Serial.print("ALS Upper Threshold : ");        Serial.println(s);
  s=displayALSLowerThresholdValue;
  Serial.print("ALS Lower Threshold : ");        Serial.println(s);
  s=displayDelayBeforeChangeValue;
  Serial.print("Delay Before Change Val : ");    Serial.println(s);
  s=displayALSCurrentValue;
  Serial.print("Disp ALS Current Val : ");       Serial.println(s);
  s=displayBacklightUpperValue;
  Serial.print("Display Upper Val : ");          Serial.println(s);
  s=displayBacklightLowerValue;
  Serial.print("Display Lower Val : ");          Serial.println(s);
  s=displayBacklightCurrentValue;
  Serial.print("Disp Backlight Current Val : "); Serial.println(s);
  s=displayDelayBeforeScrollValue;
  Serial.print("Delay Before Scroll Val : "); Serial.println(s);
  s=bAllowGestureControl;
  Serial.print("Allow Gesture Control Val : "); Serial.println(s);
  #endif
}



void connectMQTT() {
  int connection_counts = 0;
  eSENSORSTATE tmpeSENSORSTATE_STATE;
  bBrokerPresent = true;
  #ifdef DEBUG_GENERAL
  conDotCountMQTT = 0;
  #endif
  tmpeSENSORSTATE_STATE = eSENSORSTATE_STATE; // Record the state connectMQTT was entered from. 
  // Make sure we are connected to WIFI before attemping to reconnect to MQTT
  eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_MQTT;
  timer_update(); // Update timers
  if(WiFi.status() == WL_CONNECTED){
    // Loop until we're reconnected to the MQTT server
    #ifdef DEBUG_STATE_CHANGE
    SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_PENDING_MQTT,"connectMQTT");
    #endif
    eSENSORSTATE_STATE = eSENSORSTATE_PENDING_MQTT;
    #ifdef DEBUG_GENERAL
    Serial.print("Attempting MQTT connection");
    #endif
    while (!MQTTclient.connected()) {
      #ifdef DEBUG_GENERAL
      if (conDotCountMQTT > 50)
          conDotCountMQTT = 0;
      if (conDotCountMQTT == 0)
        Serial.print(".");
      conDotCountMQTT++;  
      #endif
    
      timer_update(); // Update timers
      server.handleClient();      
      //if connected, subscribe to the topic(s) we want to be notified about
      if (MQTTclient.connect((char*) clientName.c_str())) {
        // Start wifi subsystem
        WiFi.mode(WIFI_STA);  // Switch off access point and turn into station only
        //WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
        #ifdef DEBUG_GENERAL
        Serial.println();
        Serial.println("Switching to STA Mode. Now MQTT is connected.");
        #endif
        MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());        
        makeSubscriptions();
        subscribeToTopics(ptrHeadOfSensorInstances);
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_ACTIVE,"connectMQTT");
        #endif
        eSENSORSTATE_STATE = eSENSORSTATE_ACTIVE;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        updateMQTTBrokerConnectionStatus(true);
        updateWiFiNetworkConnectionStatus(true);
        updateAccessPointStatus(false);
      } else { //otherwise print failed for debugging
        #ifdef DEBUG_GENERAL
        Serial.println("\tFailed."); 
        #endif
        //abort();
      }
      
      if(WiFi.status() != WL_CONNECTED) { // Catches a lost NW whilst looking for MQTT broker
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"connectMQTT WiFi lost");
        #endif
        eSENSORSTATE_STATE = eSENSORSTATE_INIT;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        updateMQTTBrokerConnectionStatus(false);
        updateWiFiNetworkConnectionStatus(false);
        return;
      }
      
      if (eSENSORSTATE_STATE == eSENSORSTATE_INIT) // Catches the state where device is hung in MQTT pending mode with mqtt_broker_connection_attempts==0 and user sets new config via handleNetworkConfig
      {
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        timer_update();
        return;
      }  
      
      if ((connection_counts >= mqtt_broker_connection_attempts) && (mqtt_broker_connection_attempts > 0))
      {
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"connectMQTT con count");
        #endif
        eSENSORSTATE_STATE = eSENSORSTATE_INIT;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        timer_update();
        if (tmpeSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          bBrokerPresent = true; // Force programme to go back to eSENSORSTATE_INIT state if MQTT Broker conn lost after having connected to the nw and broker at least once
        else
          bBrokerPresent = false; // Force programme to go to eSENSORSTATE_NO_CONFIG State if after MQTT connection attempts made and never having made an MQTT on this nw before
        return;
      }
      
      if (mqtt_broker_connection_attempts > 0)
        connection_counts++;
      yield();  
      //delay(10);
    }
  } else { // catches a lost NW as the cause for an MQTT broker connection failure
    #ifdef DEBUG_STATE_CHANGE
    SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"connectMQTT no WiFi at start");
    #endif
    eSENSORSTATE_STATE = eSENSORSTATE_INIT;
    eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
    updateMQTTBrokerConnectionStatus(false);
    updateWiFiNetworkConnectionStatus(false);
  }
}


void makeSubscriptions(void)
{
// Fixes these : https://github.com/knolleary/pubsubclient/issues/141
//             : https://github.com/knolleary/pubsubclient/issues/98
  for (int index = 0; index < maxSubscriptions; index++)
  {
    MQTTclient.subscribe(subscriptionsArray[index]);
    for (int i=0;i<10;i++) {
      MQTTclient.loop();
      yield();
      //delay(10);
    }
  }
}


//generate unique name from MAC addr
String macToStr(const uint8_t* mac, boolean addColons){

  String result;

  for (int i = 0; i < 6; ++i) {
    if ((mac[i] & 0xF0) == 0)
      result += String(0, HEX); // stop suppression of leading zero
    result += String(mac[i], HEX);

    if (addColons && (i < 5)){
      result += ':';
    }
  }
  
  return result;
}


void timer_create(int iTimerNumber, unsigned long ulTimerPeriod, void (*callbackfn)(void))
{
  if (iTimerNumber <= MAX_TIMERS)
  {
    stiTimerArray[iTimerNumber].tmrcallback = callbackfn;
    stiTimerArray[iTimerNumber].bRunning = false;
    stiTimerArray[iTimerNumber].ulTimerPeriod = ulTimerPeriod;
    stiTimerArray[iTimerNumber].ulStartValue = 0;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Create, TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TPeriod : "));
    Serial.println(ulTimerPeriod);
    #endif
  }
}



void timer_update(void)
{
  unsigned long ulCurrentTime = millis();
  unsigned long ulElapsedTime = 0;
  
  for (int iIndex = 0; iIndex < MAX_TIMERS; iIndex++)
  {
    if (stiTimerArray[iIndex].bRunning)
    {
      ulElapsedTime = ulCurrentTime - stiTimerArray[iIndex].ulStartValue;
      /* // Argh! twos complement arithmetic, I hate it...
      if (ulCurrentTime < stiTimerArray[iIndex].ulStartValue) // Cater for UL counter wrap ~every day
        ulElapsedTime = ulCurrentTime - stiTimerArray[iIndex].ulStartValue;
      else  
        ulElapsedTime = ulCurrentTime + (ULONG_MAX - stiTimerArray[iIndex].ulStartValue);
      */
      #ifdef DEBUG_TIMER
      Serial.print(F("T Up, TNum : "));
      Serial.print(iIndex);
      Serial.print(F(", T Elapsed : "));
      Serial.println(ulElapsedTime);
      #endif
        
      if (ulElapsedTime >= stiTimerArray[iIndex].ulTimerPeriod)
      {
        stiTimerArray[iIndex].bRunning = false;
        stiTimerArray[iIndex].tmrcallback();
      }
    }
  }
}


void timer_start(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
  {
    stiTimerArray[iTimerNumber].ulStartValue = millis();
    stiTimerArray[iTimerNumber].bRunning = true;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Start , TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TStart : "));
    Serial.println(stiTimerArray[iTimerNumber].ulStartValue);
    #endif
  }
}


void timer_stop(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
    stiTimerArray[iTimerNumber].bRunning = false;
  #ifdef DEBUG_TIMER
  Serial.print(F("T Stop : "));
  Serial.println(iTimerNumber);
  #endif
}


void timer_reset(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
    stiTimerArray[iTimerNumber].ulStartValue = millis();
  #ifdef DEBUG_TIMER
  Serial.print(F("T Reset : "));
  Serial.println(iTimerNumber);
  #endif
}


boolean timer_isRunning(int iTimerNumber)
{
  return stiTimerArray[iTimerNumber].bRunning;
}


void timer_change_period(int iTimerNumber, unsigned long ulTimerPeriod)
{
  boolean bTmpRunning;
  if (iTimerNumber <= MAX_TIMERS)
  {
    bTmpRunning = stiTimerArray[iTimerNumber].bRunning;
    stiTimerArray[iTimerNumber].bRunning = false;
    stiTimerArray[iTimerNumber].ulTimerPeriod = ulTimerPeriod;
    stiTimerArray[iTimerNumber].bRunning = bTmpRunning;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Change Period, TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TPeriod : "));
    Serial.println(ulTimerPeriod);
    #endif
  }
}


  
void ledFlashTimerCallback(void)
{
  // This is called if the led flash timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In ledFlashTimerCallback()");
  #endif


  #ifdef DEBUG_LEDFLASH
  SHOW_UPDATED_LED_STATE(eLEDFLASHSTATE_STATE,eLEDFLASHSTATE_STATE,"ledFlashTimerCallback");
  Serial.print("Led Flash : ");
  Serial.print(cFlashProfiles[eLEDFLASHSTATE_STATE][iFlashSequenceIndex]);
  Serial.print(", Led Flash Ind : ");
  Serial.print(iFlashSequenceIndex);
  Serial.print(", Led Flash State : ");
  Serial.println(eLEDFLASHSTATE_STATE);
  #endif


  switch (eLEDFLASHSTATE_STATE){
    case eLEDFLASH_NO_CONFIG    :
    case eLEDFLASH_PENDING_NW   :
    case eLEDFLASH_PENDING_MQTT :
        if (cFlashProfiles[eLEDFLASHSTATE_STATE][iFlashSequenceIndex] == '1')
          port.digitalWrite(lightPin0, HIGH); // Led on
        else  
          port.digitalWrite(lightPin0, LOW); // Led off
        break;
        
    case eLEDFLASH_SEQUENCE_END : 
        port.digitalWrite(lightPin0, LOW); // Led off
        eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
        break;
        
    case eLEDFLASH_OFF : 
        iFlashSequenceIndex = 0;
        break;
        
    default : 
        break;
  }

  iFlashSequenceIndex++;
  if (iFlashSequenceIndex >= (FLASH_SEQUENCE_MAX-1))
    iFlashSequenceIndex = 0;

  //if (eLEDFLASHSTATE_STATE != eLEDFLASH_OFF)
    timer_start(LED_FLASH_TIMER);
  #ifdef DEBUG_SD
  Serial.println("In ledFlashTimerCallback");
  #endif
}


void periodicUpdateTimerCallback(void)
{
  // This is called if the Periodic Update timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In periodicUpdateTimerCallback()");
  #endif

  sendTHUpdate = true;
  sendBUpdate = true;
  sendALSUpdate = true;
  timer_start(PERIODIC_UPDATE_TIMER);
  #ifdef DEBUG_SD
  Serial.println("In periodicUpdateTimerCallback");
  #endif
}



void displayBrightnessChangeTimerCallback(void)
{
  // This is called if the Display Brightness Change Timer has timed out. This means a display threshold has been passed for long enough to mean the brightness now must change
  #ifdef DEBUG_TIMER
  Serial.println("In displayBrightnessChangeTimerCallback()");
  #endif

  switch (eALS_LEVEL_STATE){
    case eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_BRIGHT :
        displayBacklightCurrentValue = displayBacklightLowerValue;
        setBackLightLevel(displayBacklightCurrentValue);
        #ifdef DEBUG_BACKLIGHT
        Serial.println("TMR ALS Display too bright");
        #endif
        break;
        
    case eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_DIM : 
        displayBacklightCurrentValue = displayBacklightUpperValue;
        setBackLightLevel(displayBacklightCurrentValue);
        #ifdef DEBUG_BACKLIGHT
        Serial.println("TMR ALS Display too dim");
        #endif
        break;
        
    case eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK : 
        #ifdef DEBUG_BACKLIGHT
        Serial.println("TMR ALS Display ok");
        #endif
    default : 
        break;
  }

  #ifdef DEBUG_BACKLIGHT
  Serial.println("In displayBrightnessChangeTimerCallback");
  #endif
}



void displayScrollUpdateTimerCallback(void)
{
  // This is called if the Periodic timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In displayScrollUpdateTimerCallback()");
  #endif
  bScrollDisplay = true;

  timer_start(DISPLAY_SCOLL_UPDATE_TIMER);
  #ifdef DEBUG_LCD_UPDATE
  Serial.println("In displayScrollUpdateTimerCallback");
  #endif
}



void returnOK(String mess) {
  #ifdef DEBUG_WEB
  Serial.println("returnOK");  
  #endif
  if (mess.length() > 0)
    server.send(200, "text/html", mess);
  else  
    server.send(200, "text/plain", "");
}

void returnFail(String msg) {
  #ifdef DEBUG_WEB
  Serial.println("returnFail");  
  #endif
  server.send(500, "text/plain", msg + "\r\n");
}

bool loadFromSD(String path){
  String dataType = "text/plain";
  #ifdef DEBUG_WEB
  Serial.println("loadFromSD");  
  #endif
  if(path.endsWith("/")) path += "index.htm";

  if(path.endsWith(".src")) path = path.substring(0, path.lastIndexOf("."));
  else if(path.endsWith(".htm")) dataType = "text/html";
  else if(path.endsWith(".css")) dataType = "text/css";
  else if(path.endsWith(".js")) dataType = "application/javascript";
  else if(path.endsWith(".json")) dataType = "application/json";
  else if(path.endsWith(".png")) dataType = "image/png";
  else if(path.endsWith(".gif")) dataType = "image/gif";
  else if(path.endsWith(".jpg")) dataType = "image/jpeg";
  else if(path.endsWith(".ico")) dataType = "image/x-icon";
  else if(path.endsWith(".xml")) dataType = "text/xml";
  else if(path.endsWith(".pdf")) dataType = "application/pdf";
  else if(path.endsWith(".zip")) dataType = "application/zip";
  else if(path.endsWith(".png")) dataType = "image/png";

  File dataFile = SD.open(path.c_str(),SD_FILE_READ_MODE);

  if (!dataFile)
    return false;

  if (server.hasArg("download")) dataType = "application/octet-stream";

  if (server.streamFile(dataFile, dataType) != dataFile.size()) {
    #ifdef DEBUG_WEB
    Serial.println("Sent less data than expected!");
    #endif
  }

  dataFile.close();
  return true;
}




void handleNetworkConfig()
{
  String pass_response;
  String fail_response;
  char tmp_mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
  int  tmp_mqtt_broker_port;
  int  tmp_mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
  char tmp_sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
  char tmp_sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
  int  tmp_network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
  //char tmp_ap_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
  //char tmp_ap_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];

  pass_response  = "<html>";
  pass_response += "  <head>";
  pass_response += "   <title>Form submitted</title>";
  pass_response += " </head>";
  pass_response += " <body>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Sensor Configuration Home Page </b> </font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'>New configuration details now submitted</font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'><a href='index.htm'>Return to main page</a></font></p>";
  pass_response += " </body>";
  pass_response += "</html>";  

  fail_response  = "<html>";
  fail_response += "  <head>";
  fail_response += "   <title>Form not submitted</title>";
  fail_response += " </head>";
  fail_response += " <body>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Sensor Configuration Home Page </b> </font></p>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif'>Return to main page and re-submit details</font></p>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif'><a href='index.htm'>Return to main page</a></font></p>";
  fail_response += " </body>";
  fail_response += "</html>";  

  #ifdef DEBUG_WEB
  Serial.println("handleNetworkConfig");
  #endif
  String strMQTTBrokerIPAddress=server.arg("MQTTBrokerIPAddress");
  String strMQTTBrokerPort=server.arg("MQTTBrokerPort");
  String strMQTTBrokerConnectionAttempts=server.arg("MQTTBrokerConnectionAttempts");
  String strNetworkSSID=server.arg("NetworkSSID");
  String strNetworkPassword=server.arg("NetworkPassword");
  String strNetworkConnectionAttempts=server.arg("NetworkConnectionAttempts");

  strMQTTBrokerIPAddress.trim();
  strMQTTBrokerPort.trim();
  strMQTTBrokerConnectionAttempts.trim();
  strNetworkSSID.trim();
  strNetworkPassword.trim();
  strNetworkConnectionAttempts.trim();

  strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
  if (! isValidIpv4Address((char *)strMQTTBrokerIPAddress.c_str())) {
    returnOK(fail_response);
  } else {
    //strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
    if (! isValidNumber(strMQTTBrokerPort)) {
      returnOK(fail_response);
    } else {
      tmp_mqtt_broker_port = strMQTTBrokerPort.toInt();
      if (((strNetworkSSID.length() == 0)     || (strNetworkSSID.length() >= NETWORK_SSID_STRING_MAX_LEN)) || 
          ((strNetworkPassword.length() == 0) || (strNetworkPassword.length() >= NETWORK_PASSWORD_STRING_MAX_LEN))) {
        returnOK(fail_response);
      } else {
        strcpy(tmp_sta_network_ssid,strNetworkSSID.c_str());
        strcpy(tmp_sta_network_password,strNetworkPassword.c_str());

        if (! isValidNumber(strMQTTBrokerConnectionAttempts)) {
          returnOK(fail_response);
        } else {
          tmp_mqtt_broker_connection_attempts = strMQTTBrokerConnectionAttempts.toInt();
          if ((tmp_mqtt_broker_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_mqtt_broker_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
            returnOK(fail_response);
          } else {
            if (! isValidNumber(strNetworkConnectionAttempts)) {
              returnOK(fail_response);
            } else {
              tmp_network_connection_attempts = strNetworkConnectionAttempts.toInt();
              if ((tmp_network_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_network_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
                returnOK(fail_response);
              } else {
                strcpy(mqtt_broker_ip,tmp_mqtt_broker_ip);
                mqtt_broker_port = tmp_mqtt_broker_port;
                mqtt_broker_connection_attempts = tmp_mqtt_broker_connection_attempts;
                strcpy(sta_network_ssid,tmp_sta_network_ssid);
                strcpy(sta_network_password,tmp_sta_network_password);
                network_connection_attempts = tmp_network_connection_attempts;
                bBrokerPresent = true;
                // Save new network parameters
                File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
                if (f) {
                  fileWrite(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
                  f.close();
                }      
                returnOK(pass_response);
                #ifdef DEBUG_STATE_CHANGE
                SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"handleNetworkConfig");
                #endif
                eSENSORSTATE_STATE = eSENSORSTATE_INIT;
              }
            }
          }
        }
      }
    }
  }
 
  #ifdef DEBUG_WEB
  Serial.print("MQTTBrokerIPAddress : "); Serial.println(mqtt_broker_ip);
  Serial.print("MQTTBrokerPort : "); Serial.println(mqtt_broker_port);
  Serial.print("MQTTBrokerConnectionAttempts : "); Serial.println(mqtt_broker_connection_attempts);
  Serial.print("STANetworkSSID : "); Serial.println(sta_network_ssid);
  Serial.print("STANetworkPassword : "); Serial.println(sta_network_password);
  Serial.print("NetworkConnectionAttempts : "); Serial.println(network_connection_attempts);
  #endif
  return;
}



/*
 * http://www.esp8266.com/viewtopic.php?f=29&t=2153
 * 
Processing arguments of GET and POST requests is also easy enough. Let's make our sketch turn a led on or off depending on the value of a request argument.
http://<ip address>/led?state=on will turn the led ON
http://<ip address>/led?state=off will turn the led OFF
CODE: SELECT ALL
server.on("/led", []() {
  String state=server.arg("state");
  if (state == "on") digitalWrite(13, LOW);
  else if (state == "off") digitalWrite(13, HIGH);
  server.send(200, "text/plain", "Led is now " + state);
});
- See more at: http://www.esp8266.com/viewtopic.php?f=29&t=2153#sthash.7O0kU5VW.dpuf
 */

void handleNotFound(){
  #ifdef DEBUG_WEB
  Serial.println("handleNotFound");
  #endif
  if(hasSD && loadFromSD(server.uri())) return;
  String message = "SD Not Detected\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:"+server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  #ifdef DEBUG_WEB
  Serial.print(message);
  #endif
}



boolean isFloat(String tString) {
  String tBuf;
  boolean decPt = false;
  
  if(tString.charAt(0) == '+' || tString.charAt(0) == '-') tBuf = &tString[1];
  else tBuf = tString;  

  for(int x=0;x<tBuf.length();x++)
  {
    if(tBuf.charAt(x) == '.') {
      if(decPt) return false;
      else decPt = true;  
    }    
    else if(tBuf.charAt(x) < '0' || tBuf.charAt(x) > '9') return false;
  }
  return true;
}



boolean isValidNumber(String str){
   for(byte i=0;i<str.length();i++)
   {
      if(!isDigit(str.charAt(i))) return false;
   }
   return true;
} 


bool isValidIpv4Address(char *st)
{
    int num, i, len;
    char *ch;

    //counting number of quads present in a given IP address
    int quadsCnt=0;

    #ifdef DEBUG_VALIDATION
    Serial.print("Split IP: ");
    Serial.println(st);
    #endif
    len = strlen(st);

    //  Check if the string is valid
    if(len<7 || len>15)
        return false;

    ch = strtok(st, ".");

    while (ch != NULL) 
    {
        quadsCnt++;
        #ifdef DEBUG_VALIDATION
        Serial.print("Quald ");
        Serial.print(quadsCnt);
        Serial.print(" is ");
        Serial.println(ch);
        #endif

        num = 0;
        i = 0;

        //  Get the current token and convert to an integer value
        while(ch[i]!='\0')
        {
            num = num*10;
            num = num+(ch[i]-'0');
            i++;
        }

        if(num<0 || num>255)
        {
            #ifdef DEBUG_VALIDATION
            Serial.println("Not a valid ip");
            #endif
            return false;
        }

        if( (quadsCnt == 1 && num == 0) || (quadsCnt == 4 && num == 0))
        {
            #ifdef DEBUG_VALIDATION
            Serial.print("Not a valid ip, quad: ");
            Serial.print(quadsCnt);
            Serial.print(" AND/OR quad: ");
            Serial.print(quadsCnt);
            Serial.println(" is zero");
            #endif
            return false;
        }

        ch = strtok(NULL, ".");
    }

    //  Check the address string, should be n.n.n.n format
    if(quadsCnt!=4)
    {
        return false;
    }

    //  Looks like a valid IP address
    return true;
}


void checkBarometricPressure(void)
{
  unsigned long currentMillis = millis();
  float fBarometricTotal = 0.0;
  int iTempBarometricDeltaCount = 0;
  float fTempBarometricDeltaCount = 0.0;
  unsigned long tmp_barometric_new = 0;

  if (bBMPSensorFail) return;
  
  if (reportingStrategy == 0) {   
    if(currentMillis - previousBarometricMillis >= readIntervalBarometric) {
      // save the last time you read the sensor 
      previousBarometricMillis = currentMillis;   
  
      // 1 hectopascal = 0.7500616827 millimeter of mercury
      // 1 hPa = 0.7500616827 mmHg
      
      // 1 hectopascal = 1 millibar
      // 1 hPa = 1 mbar
  
      barometric_new = bmp.readPressure();
      barometric_new *= baroCalScale;
      barometric_new += baroCalOffset;
      // barometric_new = bmp.readTemperature();
      if (isnan(barometric_new)) {
        #ifdef DEBUG_GENERAL
        Serial.println("Failed to read from BMP085 sensor!");
        #endif
        newBarometricData = false;
        return;
      }

      #ifdef DEBUG_GENERAL
      if (barometric_new != barometric_old) {
        Serial.println("Rep Strat ==0");
        Serial.print(barometric_new);
        Serial.println("mBar");
        } else {
          Serial.println("Rep Strat ==0");
          Serial.println("No change to B");
        }      
       #endif

      if (barometric_new != barometric_old)
      {
        sprintf(strTempBaro,"%ld.%02ld",(unsigned long)(barometric_new/100), (unsigned long)(barometric_new - ((unsigned long)(((unsigned long)(barometric_new/100))*100))) );
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(barometricTopic, strTempBaro);        
        barometric_old = barometric_new;
        newBarometricData = true;
      }
    }
  } else {

    if (sendBUpdate == true) {
      sendBTrendUpdate = true;
      #ifdef DEBUG_GENERAL
      Serial.println("Rep Strat <> 0");
      #endif
      
      sendBUpdate = false;
      barometric_new = bmp.readPressure();

      if (isnan(barometric_new)) {
        #ifdef DEBUG_GENERAL
        Serial.println("Failed to read from BMP085 sensor!");
        #endif
        newBarometricData = false;
        return;
      }

      if (!isnan(barometric_new)) {
        barometric_new *= baroCalScale;
        barometric_new += baroCalOffset;
        if (barometric_new != barometric_old)
        {
          sprintf(strTempBaro,"%ld.%02ld",(unsigned long)(barometric_new/100), (unsigned long)(barometric_new - ((unsigned long)(((unsigned long)(barometric_new/100))*100))) );
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(barometricTopic, strTempBaro);        
          barometric_old = barometric_new;
          newBarometricData = true;
        }
      }
      
      #ifdef DEBUG_GENERAL
      Serial.print("Barometric Pressure : ");
      Serial.print(barometric_new);
      Serial.println(" mBar");
      #endif 
    }
  }

  if (((reportingStrategy == 0) &&(newBarometricData)) || (sendBTrendUpdate == true)) {
    sendBTrendUpdate = false;
    tmp_barometric_new = barometric_new;
    #if defined(DEBUG_BARO_TREND) && defined(DEBUG_BARO_FAKE_DATA)
    tmp_barometric_new =  ulDummyBarometricTrendArray[iDummyBarometricTrendArrayPointer++];
    if (iDummyBarometricTrendArrayPointer >= (sizeof(ulDummyBarometricTrendArray)/sizeof(unsigned long )))
      iDummyBarometricTrendArrayPointer = 0;
    #endif
  
    if (bFirstBarometricRead) {
      bFirstBarometricRead = false;
      for (int i = 0; i < MAX_BAROMETRIC_SAMPLES; i++)
        fBarometricRollingAverage[i] = ((float)(tmp_barometric_new/100.0));
      fOldBarometricTrendAverage = ((float)(tmp_barometric_new/100.0));
      fNewBarometricTrendAverage = ((float)(tmp_barometric_new/100.0));
    } else {
      fBarometricRollingAverage[iBarometricRollingAveragePointer++] = ((float)(tmp_barometric_new/100.0));
      for (int i = 0; i < MAX_BAROMETRIC_SAMPLES; i++)
        fBarometricTotal += fBarometricRollingAverage[i];
      fNewBarometricTrendAverage = fBarometricTotal/((float)MAX_BAROMETRIC_SAMPLES);
      fTempBarometricDeltaCount = ((fNewBarometricTrendAverage - fOldBarometricTrendAverage)/MAX_BAROMETRIC_DELTA);
      iTempBarometricDeltaCount = (int)fTempBarometricDeltaCount;
      iBarometricDeltaCount += iTempBarometricDeltaCount;
      if (iBarometricDeltaCount < BAROMETRIC_DELTA_LOWER)
        iBarometricDeltaCount = BAROMETRIC_DELTA_LOWER;
      else {
        if (iBarometricDeltaCount > BAROMETRIC_DELTA_UPPER)
          iBarometricDeltaCount = BAROMETRIC_DELTA_UPPER;
      }
      if (iTempBarometricDeltaCount == 0) {
        if (iBarometricDeltaCount > 0)
          iBarometricDeltaCount--;
        else {
          if (iBarometricDeltaCount < 0)
            iBarometricDeltaCount++;  
        }
      }
    }
    #ifdef DEBUG_BARO_TREND
    {
      char strTmp[300];
      char strTmp1[300];
      unsigned long ulTmpUL = ((unsigned long)(fNewBarometricTrendAverage*100.00));
      Serial.println();
      sprintf(strTmp,"fNewBarometricTrendAverage : %ld.%02ld",(unsigned long)(ulTmpUL/100), (unsigned long)(ulTmpUL - ((unsigned long)(((unsigned long)(ulTmpUL/100))*100))) );
      Serial.println(strTmp);
      ulTmpUL = ((unsigned long)(fOldBarometricTrendAverage*100.00));
      sprintf(strTmp,"fOldBarometricTrendAverage : %ld.%02ld",(unsigned long)(ulTmpUL/100), (unsigned long)(ulTmpUL - ((unsigned long)(((unsigned long)(ulTmpUL/100))*100))) );
      Serial.println(strTmp);        

      sprintf(strTmp,"fTempBarometricDeltaCount : %s",fmtDouble(fTempBarometricDeltaCount, 4, strTmp1, 300));
      Serial.println(strTmp);        
      
      Serial.print("fBarometricRollingAverage Array : ");
      for(int i = 0; i < MAX_BAROMETRIC_SAMPLES; i++)
      {
        ulTmpUL = ((unsigned long)(fBarometricRollingAverage[i]*100.00));
        sprintf(strTmp,"%ld.%02ld,",(unsigned long)(ulTmpUL/100), (unsigned long)(ulTmpUL - ((unsigned long)(((unsigned long)(ulTmpUL/100))*100))) );
        Serial.print(strTmp);          
      }
      Serial.println();          
    }
    #endif
    fOldBarometricTrendAverage = fNewBarometricTrendAverage;
     if (iBarometricRollingAveragePointer >= MAX_BAROMETRIC_SAMPLES)
      iBarometricRollingAveragePointer = 0;
    #ifdef DEBUG_BARO_TREND
    {
      char strTmp[300];
      sprintf(strTmp,"tmp_barometric_new : %ld.%02ld",(unsigned long)(tmp_barometric_new/100), (unsigned long)(tmp_barometric_new - ((unsigned long)(((unsigned long)(tmp_barometric_new/100))*100))) );
      Serial.println(strTmp);
      sprintf(strTmp,"iBarometricRollingAveragePointer : %d",iBarometricRollingAveragePointer);
      Serial.println(strTmp);
      sprintf(strTmp,"iTempBarometricDeltaCount : %d",iTempBarometricDeltaCount);
      Serial.println(strTmp);
      sprintf(strTmp,"iBarometricDeltaCount : %d",iBarometricDeltaCount);
      Serial.println(strTmp);
      Serial.println();
    }
    #endif
  }
}

#if defined(DEBUG_BARO_TREND) && defined(DEBUG_GENERAL)
//
// Format a floating point value with number of decimal places.
// The 'precision' parameter is a number from 0 to 6 indicating the desired decimal places.
// The 'buf' parameter points to a buffer to receive the formatted string.  This must be
// sufficiently large to contain the resulting string.  The buffer's length may be
// optionally specified.  If it is given, the maximum length of the generated string
// will be one less than the specified value.
//
// example: fmtDouble(3.1415, 2, buf); // produces 3.14 (two decimal places)
//
char *fmtDouble(double val, byte precision, char *buf, unsigned bufLen)
{
  char *tmpBufPtr = buf;
  
 if (!buf || !bufLen)
   return NULL;

 // limit the precision to the maximum allowed value
 const byte maxPrecision = 6;
 if (precision > maxPrecision)
   precision = maxPrecision;

 if (--bufLen > 0)
 {
   // check for a negative value
   if (val < 0.0)
   {
     val = -val;
     *buf = '-';
     bufLen--;
   }

   // compute the rounding factor and fractional multiplier
   double roundingFactor = 0.5;
   unsigned long mult = 1;
   for (byte i = 0; i < precision; i++)
   {
     roundingFactor /= 10.0;
     mult *= 10;
   }

   if (bufLen > 0)
   {
     // apply the rounding factor
     val += roundingFactor;

     // add the integral portion to the buffer
     unsigned len = fmtUnsigned((unsigned long)val, buf, bufLen, precision);
     buf += len;
     bufLen -= len;
   }

   // handle the fractional portion
   if ((precision > 0) && (bufLen > 0))
   {
     *buf++ = '.';
     if (--bufLen > 0)
       buf += fmtUnsigned((unsigned long)((val - (unsigned long)val) * mult), buf, bufLen, precision);
   }
 }

 // null-terminate the string
 *buf = '\0';
 return tmpBufPtr;
}


//
// Produce a formatted string in a buffer corresponding to the value provided.
// If the 'width' parameter is non-zero, the value will be padded with leading
// zeroes to achieve the specified width.  The number of characters added to
// the buffer (not including the null termination) is returned.
//
unsigned fmtUnsigned(unsigned long val, char *buf, unsigned bufLen, byte width)
{
 if (!buf || !bufLen)
   return(0);

 // produce the digit string (backwards in the digit buffer)
 char dbuf[10];
 unsigned idx = 0;
 while (idx < sizeof(dbuf))
 {
   dbuf[idx++] = (val % 10) + '0';
   if ((val /= 10) == 0)
     break;
 }

 // copy the optional leading zeroes and digits to the target buffer
 unsigned len = 0;
 byte padding = (width > idx) ? width - idx : 0;
 char c = '0';
 while ((--bufLen > 0) && (idx || padding))
 {
   if (padding)
     padding--;
   else
     c = dbuf[--idx];
   *buf++ = c;
   len++;
 }

 // add the null termination
 *buf = '\0';
 return(len);
}
#endif


void checkALSValue(void)
{
  unsigned long currentMillis = millis();  
  #ifdef DEBUG_BACKLIGHT
  //Serial.println("In checkALSValue()");
  #endif
  
  displayALSCurrentValue = (unsigned long) ambientLightSensor.readLightLevel();
  strAmbientLightLevel  = String(displayALSCurrentValue);
  ambient_light_sensor_new = displayALSCurrentValue;
  
  if ((displayBacklightCurrentValue == displayBacklightLowerValue) && 
      (displayALSCurrentValue > displayALSUpperThresholdValue)) {
      if (eALS_LEVEL_STATE != eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_DIM) {  
        timer_reset(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
        timer_start(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
        eALS_LEVEL_STATE = eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_DIM;          
        #ifdef DEBUG_BACKLIGHT
        Serial.println("ALS Display too dim");
        Serial.print("ASL Val : ");
        Serial.println(displayALSCurrentValue);
        #endif
      }
  } else {
    if ((displayBacklightCurrentValue == displayBacklightLowerValue) && 
        (displayALSCurrentValue <= displayALSUpperThresholdValue)) {
        if (eALS_LEVEL_STATE != eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK) {
          timer_stop(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
          timer_reset(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
          eALS_LEVEL_STATE = eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK;          
          #ifdef DEBUG_BACKLIGHT
          Serial.println("ALS Display ok.");
          Serial.print("ASL Val : ");
          Serial.println(displayALSCurrentValue);
          #endif
        }
    } else {
      if ((displayBacklightCurrentValue == displayBacklightUpperValue) && 
          (displayALSCurrentValue < displayALSLowerThresholdValue)) {
          if (eALS_LEVEL_STATE != eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_BRIGHT) {
            timer_reset(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
            timer_start(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
            eALS_LEVEL_STATE = eALS_LEVEL_CONTROL_STATE_DISPLAY_TOO_BRIGHT;          
            #ifdef DEBUG_BACKLIGHT
            Serial.println("ALS Display too bright");
            Serial.print("ASL Val : ");
            Serial.println(displayALSCurrentValue);
            #endif
          }
      } else {
        if ((displayBacklightCurrentValue == displayBacklightUpperValue) && 
            (displayALSCurrentValue >= displayALSLowerThresholdValue)) {
            if (eALS_LEVEL_STATE != eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK) {
              timer_stop(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
              timer_reset(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
              eALS_LEVEL_STATE = eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK;          
              #ifdef DEBUG_BACKLIGHT
              Serial.println("ALS Display ok");
              Serial.print("ASL Val : ");
              Serial.println(displayALSCurrentValue);
              #endif
            }
        } else {
          // This is an error condition
          timer_stop(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
          timer_reset(DISPLAY_BRIGHTNESS_CHANGE_TIMER);
          eALS_LEVEL_STATE = eALS_LEVEL_CONTROL_STATE_DISPLAY_BRIGHTNESS_OK;          
          #ifdef DEBUG_BACKLIGHT
          Serial.println("ALS Error condition");
          Serial.print("ASL Val : ");
          Serial.println(displayALSCurrentValue);
          #endif
        }
      }
    }
  }

  if (reportingStrategy == 0) {   
    if(currentMillis - previousALSMillis >= readIntervalALS) {
      // save the last time you read the sensor 
      previousALSMillis = currentMillis;   

      #ifdef DEBUG_GENERAL
      if (ambient_light_sensor_new != ambient_light_sensor_old) {
        Serial.println("ALS Rep Strat ==0");
        Serial.print(ambient_light_sensor_new);
        Serial.println("Lux");
        } else {
          Serial.println("ALS Rep Strat ==0");
          Serial.println("No change to ALS");
        }      
       #endif

      if (ambient_light_sensor_new != ambient_light_sensor_old)
      {
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(ambientLightSensorTopic, strAmbientLightLevel.c_str());        
        ambient_light_sensor_old = ambient_light_sensor_new;
      }
    }
  } else {
    if (sendALSUpdate == true) {
      sendALSUpdate = false;
      #ifdef DEBUG_GENERAL
      Serial.println("ALS Rep Strat <> 0");
      #endif

      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
        MQTTclient.publish(ambientLightSensorTopic, strAmbientLightLevel.c_str());        
          
      #ifdef DEBUG_GENERAL
      Serial.print("ALS : ");
      Serial.print(ambient_light_sensor_new);
      Serial.println(" Lux");
      #endif
    }
  }
  ambient_light_sensor_old = ambient_light_sensor_new;
}




void updateAccessPointStatus(boolean state)
{
  lcd.setCursor(11, 0);
  if (state)
    lcd.write(AccessPointOn);
  else
    lcd.writeStr(" ");
    //lcd.write(AccessPointOff);
}


void updateWiFiNetworkConnectionStatus(boolean state)
{
  lcd.setCursor(12, 0);
  if (state)
    lcd.write(WiFiOn);
  else
    lcd.write(WiFiOff);
}


void updateMQTTBrokerConnectionStatus(boolean state)
{
  lcd.setCursor(13, 0);
  if (state)
    lcd.write(MQTTOn);
  else
    lcd.write(MQTTOff);
}


void loadChars(void)
{
    int max_chars = sizeof(binaryArray)/sizeof(binaryArrayType);
    for (unsigned char p = 0; p < max_chars ; p++)
    {
      lcd.createChar((uint8_t)p, (uint8_t *)&binaryArray[p]);
    }
}



void updateTimeDate(void)
{
  #ifdef DEBUG_DT
  char tmpTimeDateStr[sizeof(strDateFormat) + sizeof(strTimeFormat) + 30];
  #endif
  char strTmp1[40+1];
  char strTmp2[40+1];
  String strTempDate;
  String strTempTime;

  RTC.read(tm);
  if ((tm_old.Minute != tm.Minute) || (bNewTimeDateSet))
  {
      #ifdef DEBUG_DT
      sprintf(tmpTimeDateStr,"%02d:%02d %02d/%02d/%02d",tm.Hour,tm.Minute,tm.Day,tm.Month,tmYearToCalendar(tm.Year)-2000);
      Serial.print("Time Date :  ");
      Serial.println(tmpTimeDateStr);
      #endif
      tm_old = tm;
      sprintf(strTmp1,"%02d:%02d",tm.Hour,tm.Minute);
      sprintf(strTmp2,"%02d/%02d/%02d",tm.Day,tm.Month,tmYearToCalendar(tm.Year)-2000);
      strTempTime = strTmp1;
      strTempDate = strTmp2;
      if ((strTempTime == "45:165") || (strTempDate == "165/165/165") || (strstr(strTmp1,"165")) || (strstr(strTmp2,"165")))
        return;
      lcd.setCursor(15,0);
      lcd.writeStr(strTmp1); 
      lcd.setCursor(12,1);
      lcd.writeStr(strTmp2); 
      bNewTimeDateSet = false;
  }
}




int extractTime(char *strTime, int *Hours, int *Minutes)
{
  char hours[10], minutes[10];
  int iParmCount = parseString((char *)":", strTime, hours, minutes);
  if (iParmCount > 0) {
    String H(hours), M(minutes);
    H.trim();
    M.trim();
    #ifdef DEBUG_PARSER
    {
      char tmpStr[200];
      sprintf(tmpStr,"extractTime : %s:%s, PCount %d",H.c_str(),M.c_str(),iParmCount);
      Serial.println(tmpStr);
    }
    #endif
    *Hours   = H.toInt();
    *Minutes = M.toInt();
  }
  return iParmCount;
}


int extractDate(char *strDate, int *Days, int *Months, int *Years)
{
  char days[10], months[10], years[10];
  int iParmCount = parseString((char *)"/", strDate, days, months, years);
  if (iParmCount > 0) {
    String D(days), M(months), Y(years);
    D.trim();
    M.trim();
    Y.trim();
    #ifdef DEBUG_PARSER
    {
      char tmpStr[200];
      sprintf(tmpStr,"extractDate : %s/%s/%s, PCount %d",D.c_str(),M.c_str(),Y.c_str(),iParmCount);
      Serial.println(tmpStr);
    }
    #endif
    *Days   = D.toInt();
    *Months = M.toInt();
    *Years  = Y.toInt();
  }
  return iParmCount;
}



int parseString(char *delimiter, char *source, ...)
{
   va_list arg_ptr;
   int argCount = 0;
   int args = 0;
   char *srcPtr = source;
   char *destPtr;

   if (strlen(source) > 1) {
     va_start(arg_ptr, source);
     while(*srcPtr)
     {
       destPtr = va_arg(arg_ptr, char *);
       while((*srcPtr) && (*srcPtr != *delimiter))
       {
        *destPtr++ = *srcPtr++;
        *destPtr = '\0';
       }
       if ((*srcPtr == *delimiter) || (*srcPtr == '\0')){
         if (*srcPtr == *delimiter) srcPtr++;
         argCount++;
       }
     }
     va_end(arg_ptr);
   }
   return argCount;
}



void setBackLightLevel(uint8_t blVal)
{
    Wire.beginTransmission(BACKLIGHT_I2C_ADDRESS);
    Wire.write((byte)(blVal));
    Wire.endTransmission();
}


sSensorInstance * addItem(sSensorInstance **Head, const char *sSensorName, const char *sTemperatureTopic, const char *sHumidityTopic, const char *sTemp, const char *sHumid, bool bExtSrc)
{
  sSensorInstance *Tail = (sSensorInstance *) malloc(sizeof(sSensorInstance));
  sSensorInstance *ptrSITmp;
  if (!(Tail == NULL)) {
    strcpy(Tail->strSensorName,sSensorName);
    strcpy(Tail->strTemperatureTopic,sTemperatureTopic);
    strcpy(Tail->strHumidityTopic,sHumidityTopic);
    strcpy(Tail->strTemperature,sTemp);
    strcpy(Tail->strHumidity,sHumid);
    Tail->bExternalSource = bExtSrc;
    if (*Head == NULL) {  // Uninitialised head pointer
      Tail->next     = Tail;
      Tail->previous = Tail;
      *Head = Tail;
    } else {            // Entry already exists in the list, add to the end of the list
      ptrSITmp = (*Head)->previous;
      ptrSITmp->next = Tail;
      Tail->next = *Head;
      Tail->previous = ptrSITmp;
      (*Head)->previous = Tail;
    }
  } else {  // No memory available to allocate
    return ((sSensorInstance *)NULL);
  }
  return (sSensorInstance *) Tail;
}


boolean sensorListEnd(sSensorInstance *Head, sSensorInstance *Tail)
{
  return (((Head == Tail->next) && (Head != Tail)) || ((Head == Tail->next) && (Head == Tail->previous) && (Head == Tail)));
}


boolean sensorListBegining(sSensorInstance *Head, sSensorInstance *Tail)
{
  return (Head == Tail);
}


sSensorInstance * sensorListGetNext(sSensorInstance **tmpPtr)
{
  *tmpPtr = (*tmpPtr)->next;
  return (*tmpPtr);
}


sSensorInstance * sensorListGetPrevious(sSensorInstance **tmpPtr)
{
  *tmpPtr = (*tmpPtr)->previous;
  return (*tmpPtr);
}


void subscribeToTopics(sSensorInstance *Head)
{
// Fixes these : https://github.com/knolleary/pubsubclient/issues/141
//             : https://github.com/knolleary/pubsubclient/issues/98

  sSensorInstance *tmpPtr= ptrHeadOfSensorInstances;
  
  tmpPtr = Head;
  if (Head != NULL) {
    do {
      sensorListGetNext(&tmpPtr);
      if (tmpPtr->bExternalSource) {
        #ifdef DEBUG_LCD_UPDATE  
        if (!MQTTclient.subscribe(tmpPtr->strTemperatureTopic)) {
          Serial.print("Failed to Subscribe to : ");
          Serial.println(tmpPtr->strTemperatureTopic);
        }
        #else  
          MQTTclient.subscribe(tmpPtr->strTemperatureTopic);
        #endif
    
        #ifdef DEBUG_LCD_UPDATE  
          Serial.print("Subscribed to : ");
          Serial.println(tmpPtr->strTemperatureTopic);
        #endif

        for (int i=0;i<10;i++) {
          MQTTclient.loop();
          yield();
          //delay(10);
        }
        #ifdef DEBUG_LCD_UPDATE  
        if (!MQTTclient.subscribe(tmpPtr->strHumidityTopic)) {
          Serial.print("Failed to Subscribe to : ");
          Serial.println(tmpPtr->strHumidityTopic);
        }
        #else  
          MQTTclient.subscribe(tmpPtr->strHumidityTopic);
        #endif
        
        #ifdef DEBUG_LCD_UPDATE  
          Serial.print("Subscribed to : ");
          Serial.println(tmpPtr->strHumidityTopic);
        #endif    
        for (int i=0;i<10;i++) {
          MQTTclient.loop();
          yield();
          //delay(10);
        }
      }
    } while (!sensorListBegining(Head,tmpPtr));
  }
  #ifdef DEBUG_LCD_UPDATE  
    Serial.println("In subscribeToTopics");
  #endif
}



void readConfigurableSensorInput(void){
  String sSensorName;
  String sTemperatureTopic;
  String sHumidityTopic;
  bool bOk = true;
  int index = 0;

  File f = SD.open(CONFIGURABLE_SENSOR_INPUT_FILE, SD_FILE_READ_MODE);
  if (!f) {
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read Config Sensor Input Vals.");
    #endif
    return;
  } else {
/*
    addItem(&ptrHeadOfSensorInstances, "Study", temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, false);
    addItem(&ptrHeadOfSensorInstances, "Garage", temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, true);
    addItem(&ptrHeadOfSensorInstances, "Living Room", temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, true);
    addItem(&ptrHeadOfSensorInstances, "Kitchen", temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, true);
    addItem(&ptrHeadOfSensorInstances, "Hall", temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, true);
    addItem(&ptrHeadOfSensorInstances, "Guest Bedroom", temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, true);
    addItem(&ptrHeadOfSensorInstances, "Master Bedroom", temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, true);
*/    

      if (addItem(&ptrHeadOfSensorInstances, LOCAL_TEMP_AND_HUMIDITY_STRING, temperatureTopic, humidityTopic, defaultTempStr, defaultHumidStr, false) != NULL) {
        #ifdef DEBUG_LCD_UPDATE
        Serial.print("From : "); Serial.println(CONFIGURABLE_SENSOR_INPUT_FILE);
        #endif
        index = 1;
        bOk = true;
        while ((f.position() < f.size()) && (bOk))
        {
          sSensorName=f.readStringUntil('\n'); sSensorName.trim();
          if (sSensorName.length() > MAX_SYSTEM_LCD_COLUMNS) 
                sSensorName.remove(MAX_SYSTEM_LCD_COLUMNS);
          if (f.position() < f.size()) {
            sTemperatureTopic=f.readStringUntil('\n'); sTemperatureTopic.trim();
          }
          if (f.position() < f.size()) {
            sHumidityTopic=f.readStringUntil('\n'); sHumidityTopic.trim();
          }
          //addItem(&ptrHeadOfSensorInstances, sSensorName.c_str(), sTemperatureTopic.c_str(), sHumidityTopic.c_str(), defaultTempStr, defaultHumidStr, true);
          
          if ((sSensorName.length() > 0) &&
              ((sTemperatureTopic.length() > 0) && (sTemperatureTopic.length() <= (MQTT_MAX_PACKET_SIZE + 1))) &&
              ((sHumidityTopic.length()    > 0) && (sHumidityTopic.length()    <= (MQTT_MAX_PACKET_SIZE + 1))) &&
              (addItem(&ptrHeadOfSensorInstances, sSensorName.c_str(), sTemperatureTopic.c_str(), sHumidityTopic.c_str(), defaultTempStr, defaultHumidStr, true) != NULL)) {
              index++;
          } else 
              bOk = false;
          
          #ifdef DEBUG_LCD_UPDATE
          Serial.println(sSensorName);
          Serial.println(sTemperatureTopic);
          Serial.println(sHumidityTopic);
          #endif
        }
      }
      f.close();
      yield();
  }
  
  #ifdef DEBUG_LCD_UPDATE
  {
    sSensorInstance *tmpPtr = ptrHeadOfSensorInstances;
    int x = 1;
    Serial.print("Total Entries # : "); Serial.println(index);
    do {
      Serial.print("Entry #         : "); Serial.println(x);
      Serial.print("Sensor Name     : "); Serial.println(tmpPtr->strSensorName);
      Serial.print("Temp Topic      : "); Serial.println(tmpPtr->strTemperatureTopic);
      Serial.print("Humi Topic      : "); Serial.println(tmpPtr->strHumidityTopic);
      x++;
      sensorListGetNext(&tmpPtr);
      yield();
    } while (!sensorListBegining(ptrHeadOfSensorInstances,tmpPtr));
  }
  #endif
}




void updateDisplay(void)
{
  #ifdef DEBUG_LCD_UPDATE  
    //Serial.println("In updateDisplay");
  #endif
  if (newBarometricData == true){
    lcd.setCursor(0,1);
    lcd.writeStr("       "); 
    lcd.setCursor(0,1);
    lcd.writeStr(strTempBaro); 
    newBarometricData = false;
    if (iBarometricDeltaCount > 0)
      printBarometricTrend(eRISING);
    else if (iBarometricDeltaCount == 0)
      printBarometricTrend(eSTABLE);
    else
      printBarometricTrend(eFALLING);
  }

  if (tmpSensorInstancePtr != NULL) {
    if (bScrollDisplay)
    {
      bScrollDisplay = false;
      lcd.setCursor(0,2);
      lcd.writeStr("                   "); 
      lcd.setCursor(0,2);
      lcd.writeStr((const char *) (tmpSensorInstancePtr->strSensorName)); 
  
      lcd.setCursor(2,3);
      lcd.writeStr("     "); 
      lcd.setCursor(2,3);
      lcd.writeStr((const char *) (tmpSensorInstancePtr->strTemperature)); 
  
      lcd.setCursor(14,3);
      lcd.writeStr("     "); 
      lcd.setCursor(14,3);
      lcd.writeStr((const char *) (tmpSensorInstancePtr->strHumidity)); 
  
      sensorListGetNext(&tmpSensorInstancePtr);
      #ifdef DEBUG_LCD_UPDATE  
        Serial.println("In updateDisplay LL");
      #endif
    }
  } else {
    if (bOneOff) {
      bOneOff = false;
      lcd.setCursor(0,2);
      lcd.writeStr("                   "); 
      lcd.setCursor(0,2);
      lcd.writeStr((const char *) LOCAL_TEMP_AND_HUMIDITY_STRING); 
    }
    if (bNewTemperatureHumidityData) {
      String s;  

      bNewTemperatureHumidityData = false;
      lcd.setCursor(2,3);
      lcd.writeStr("     "); 
      lcd.setCursor(2,3);
      s = temp_c_new;
      lcd.writeStr((const char *) s.c_str()); 
  
      lcd.setCursor(14,3);
      lcd.writeStr("     "); 
      lcd.setCursor(14,3);
      s = humidity_new;
      lcd.writeStr((const char *) s.c_str()); 
      #ifdef DEBUG_LCD_UPDATE  
        Serial.println("In updateDisplay No LL");
      #endif
    }
  }
}  



void readAPDS9960(boolean *bGestureAvailableFlag, uint8_t *uiGestureValue)
{
  *bGestureAvailableFlag = false;
  if (bGestureSensorFail) return;
  if (gestureSensor.gestureAvailable()) {
    *uiGestureValue = gestureSensor.read();
    gestureSensor.gestureClear();
    *bGestureAvailableFlag = true;
  }
  //delay(1000);
}




void handleGesture(void) {
  if (bGestureSensorFail) return;
  if (bAllowGestureControl) {
    if ( bGestureAvailableFlag ) {
      bGestureAvailableFlag = false;
      switch ( uiGestureValue ) {
        case APDS9960_GVAL_DOWN:  // This is because the sensor is rotated through 180 degrees to house in the actual enclosure
          #ifdef DEBUG_GESTURE
          Serial.println("UP");
          #endif
          if ((eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) && 
              (bAllowConfigurableGestureControl) && 
              (strlen(ConfigurableGestureInstanceArray[eGESTURE_UP].strGestureTopic) > 0) && 
              (strlen(ConfigurableGestureInstanceArray[eGESTURE_UP].strGesturePayload) > 0))
              MQTTclient.publish(ConfigurableGestureInstanceArray[eGESTURE_UP].strGestureTopic,ConfigurableGestureInstanceArray[eGESTURE_UP].strGesturePayload);  
          break;
        case APDS9960_GVAL_UP:  // This is because the sensor is rotated through 180 degrees to house in the actual enclosure
          #ifdef DEBUG_GESTURE
          Serial.println("DOWN");
          #endif
          if ((eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) && 
              (bAllowConfigurableGestureControl) && 
              (strlen(ConfigurableGestureInstanceArray[eGESTURE_DOWN].strGestureTopic) > 0) && 
              (strlen(ConfigurableGestureInstanceArray[eGESTURE_DOWN].strGesturePayload) > 0))
              MQTTclient.publish(ConfigurableGestureInstanceArray[eGESTURE_DOWN].strGestureTopic,ConfigurableGestureInstanceArray[eGESTURE_DOWN].strGesturePayload);  
          break;
        case APDS9960_GVAL_RIGHT:  // This is because the sensor is rotated through 180 degrees to house in the actual enclosure
          timer_reset(DISPLAY_SCOLL_UPDATE_TIMER);
          sensorListGetPrevious(&tmpSensorInstancePtr);
          sensorListGetPrevious(&tmpSensorInstancePtr);
          bScrollDisplay = true;
          updateDisplay();       
          #ifdef DEBUG_GESTURE
          Serial.println("LEFT");
          #endif
          break;
        case APDS9960_GVAL_LEFT: // This is because the sensor is rotated through 180 degrees to house in the actual enclosure
          timer_reset(DISPLAY_SCOLL_UPDATE_TIMER);
          //sensorListGetNext(&tmpSensorInstancePtr);
          bScrollDisplay = true;
          updateDisplay();       
          #ifdef DEBUG_GESTURE
          Serial.println("RIGHT");
          #endif
          break;
        case APDS9960_GVAL_NEAR:
          #ifdef DEBUG_GESTURE
          Serial.println("NEAR");
          #endif
          tmpSensorInstancePtr = ptrHeadOfSensorInstances;
          timer_reset(DISPLAY_SCOLL_UPDATE_TIMER);
          bScrollDisplay = true;
          updateDisplay();       
          break;
        case APDS9960_GVAL_FAR:
          #ifdef DEBUG_GESTURE
          Serial.println("FAR");
          #endif
          break;
        case APDS9960_GVAL_ERROR:
          #ifdef DEBUG_GESTURE
          Serial.println("ERROR");
          #endif
          break;
        case APDS9960_GVAL_NONE:
          #ifdef DEBUG_GESTURE
          Serial.println("None");
          #endif
          break;
        default: ;
          #ifdef DEBUG_GESTURE
          Serial.println("Unknown State.");
          #endif 
        }
    }
  }
}


void readConfigurableUpDownGestureInput(void){
  String sGestureTopic;
  String sGesturePayload;
  bool bOk = true;
  bool bDone = false;
  int index = 0;

  File f = SD.open(CONFIGURABLE_UP_DOWN_GESTURE_INPUT_FILE, SD_FILE_READ_MODE);
  if (!f) {
    #ifdef DEBUG_GESTURE
    Serial.println("Failed to read Up Down Gesture Configurable Input Vals.");
    #endif
    bAllowConfigurableGestureControl = false;
    return;
  } else {
    #ifdef DEBUG_GESTURE
    Serial.print("From : "); Serial.println(CONFIGURABLE_UP_DOWN_GESTURE_INPUT_FILE);
    #endif
    index = 1;
    bOk   = true;
    bDone = false;
    while ((f.position() < f.size()) && (bOk) && (!bDone))
    {
      if (f.position() < f.size()) {
        sGestureTopic=f.readStringUntil('\n'); sGestureTopic.trim();
      } else {
        bOk = false;
        bAllowConfigurableGestureControl = false;
      }
      if (f.position() < f.size()) {
        sGesturePayload=f.readStringUntil('\n'); sGesturePayload.trim();
      } else {
        bOk = false;
        bAllowConfigurableGestureControl = false;
      }
      
      if (((sGestureTopic.length()   > 0) && (sGestureTopic.length()   <= (MAX_TOPIC_STRING + 1))) && (bOk) &&
          ((sGesturePayload.length() > 0) && (sGesturePayload.length() <= (MAX_MQTT_PAYLOAD_STRING + 1)))) {
          strcpy(ConfigurableGestureInstanceArray[index-1].strGestureTopic,sGestureTopic.c_str());  
          strcpy(ConfigurableGestureInstanceArray[index-1].strGesturePayload,sGesturePayload.c_str());  
          index++;
          if (index > MAX_CONFIGURABLE_GESTURE_INSTANCES) {
            bDone = true;
            bAllowConfigurableGestureControl = true;
          }
      } else {
          bOk = false;
          bAllowConfigurableGestureControl = false;
      }
      #ifdef DEBUG_GESTURE
      Serial.println(sGestureTopic);
      Serial.println(sGesturePayload);
      #endif
    } 
    f.close();
    yield();
  }
  
  #ifdef DEBUG_GESTURE
  {
    for(int x = 0; x < MAX_CONFIGURABLE_GESTURE_INSTANCES; x++) {
      Serial.print("Entry #         : "); Serial.println(x);
      Serial.print("Topic Name      : "); Serial.println(ConfigurableGestureInstanceArray[x].strGestureTopic);
      Serial.print("Payload Value   : "); Serial.println(ConfigurableGestureInstanceArray[x].strGesturePayload);
    }
  }
  #endif
}



void checkButton(void)
{
  if(myButton0.update() && myButton0.read() == HIGH){ // Read button status and check for HIGH or LOW (button uses a pull up resistor)
  #ifdef DEBUG_BUTTONS
  Serial.println("Button");
  #endif  
    if(isOn0 == false){
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) 
        MQTTclient.publish(buttonTopic, "Pressed");
      #ifdef DEBUG_BUTTONS
      Serial.println("B0 Pressed");
      #endif
      isOn0 = true;

      if ((eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) && 
          (bAllowConfigurableButtonControl) && 
          (strlen(ConfigurableButtonInstanceArray[eBUTTON_0_ON].strButtonTopic) > 0) && 
          (strlen(ConfigurableButtonInstanceArray[eBUTTON_0_ON].strButtonPayload) > 0) &&
          ConfigurableButtonInstanceArray[eBUTTON_0_ON].bButtonAssigned)
          MQTTclient.publish(ConfigurableButtonInstanceArray[eBUTTON_0_ON].strButtonTopic,ConfigurableButtonInstanceArray[eBUTTON_0_ON].strButtonPayload);  
    } else {
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) 
        MQTTclient.publish(buttonTopic, "Released");
      #ifdef DEBUG_BUTTONS
      Serial.println("B0 Released");
      #endif
      isOn0 = false;

      if ((eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) && 
          (bAllowConfigurableButtonControl) && 
          (strlen(ConfigurableButtonInstanceArray[eBUTTON_0_OFF].strButtonTopic) > 0) && 
          (strlen(ConfigurableButtonInstanceArray[eBUTTON_0_OFF].strButtonPayload) > 0) &&
          ConfigurableButtonInstanceArray[eBUTTON_0_OFF].bButtonAssigned)
          MQTTclient.publish(ConfigurableButtonInstanceArray[eBUTTON_0_OFF].strButtonTopic,ConfigurableButtonInstanceArray[eBUTTON_0_OFF].strButtonPayload);  
    }
  }


  if(myButton1.update()) {
    #ifdef DEBUG_BUTTONS
    if ((myButton1.read() != HIGH) && (myButton1.fell()))
    {
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) {
        MQTTclient.publish(buttonTopic1, "Pressed");
        if ((bAllowConfigurableButtonControl) && 
            (strlen(ConfigurableButtonInstanceArray[eBUTTON_1].strButtonTopic) > 0) && 
            (strlen(ConfigurableButtonInstanceArray[eBUTTON_1].strButtonPayload) > 0) && 
            ConfigurableButtonInstanceArray[eBUTTON_1].bButtonAssigned)
            MQTTclient.publish(ConfigurableButtonInstanceArray[eBUTTON_1].strButtonTopic,ConfigurableButtonInstanceArray[eBUTTON_1].strButtonPayload);  
      }
      isOn1 = true;
      Serial.println("Button 1 Pressed");
    }
    if ((myButton1.read() == HIGH) && (myButton1.rose()))
    {
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
        MQTTclient.publish(buttonTopic1, "Released");
      isOn1 = false;
      Serial.println("Button 1 released");
    }
    #else
    if ((myButton1.read() != HIGH) && (myButton1.fell()))
    {
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) {
        MQTTclient.publish(buttonTopic1, "Pressed");
        if ((bAllowConfigurableButtonControl) &&  
            (strlen(ConfigurableButtonInstanceArray[eBUTTON_1].strButtonTopic) > 0) && 
            (strlen(ConfigurableButtonInstanceArray[eBUTTON_1].strButtonPayload) > 0))
            MQTTclient.publish(ConfigurableButtonInstanceArray[eBUTTON_1].strButtonTopic,ConfigurableButtonInstanceArray[eBUTTON_1].strButtonPayload);  
      }
      isOn1 = true;
    }
    if ((myButton1.read() == HIGH) && (myButton1.rose()))
    {
      if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
        MQTTclient.publish(buttonTopic1, "Released");
      isOn1 = false;
    }
    #endif  
  }

  if(myCloseLoggingFileButton.update())
    #ifdef DEBUG_BUTTONS
    {
      if ((myCloseLoggingFileButton.read() != HIGH) && (myCloseLoggingFileButton.fell()))
      {
        if (currentLoggingStatus == eLoggingActive)
          isOncloseLoggingFilebuttonPin = true;
        Serial.println("Close Logging Pressed : ");
        Serial.println(isOncloseLoggingFilebuttonPin);
      }
    }
    #else
    if ((myCloseLoggingFileButton.read() != HIGH) && (myCloseLoggingFileButton.fell()) && (currentLoggingStatus == eLoggingActive))
      isOncloseLoggingFilebuttonPin = true;
    #endif  

  if(myOpenLoggingFileButton.update())
    #ifdef DEBUG_BUTTONS
    {
      if ((myOpenLoggingFileButton.read() != HIGH) && (myOpenLoggingFileButton.fell()))
      {
        if (currentLoggingStatus == eLoggingInactive)
          isOnopenLoggingFilebuttonPin = true;
        Serial.println("Open Logging Pressed : ");
        Serial.println(isOnopenLoggingFilebuttonPin);
      }
    }
    #else
    if ((myOpenLoggingFileButton.read() != HIGH) && (myOpenLoggingFileButton.fell()) && (currentLoggingStatus == eLoggingInactive))
      isOnopenLoggingFilebuttonPin = true;
    #endif  
}



void updateLogging(void)
{
  char tmpBuf[20+1];
  char tmpLogFileStr[1000];

  #ifdef DEBUG_LOGGING
  iLoggingPeriodInMinutes = 1;
  #endif
  
  RTC.read(logging_tm);
  iOldLoggingTimeInMinutes = CONVERT_TO_MINUTES(old_logging_tm);
  iNewLoggingTimeInMinutes = CONVERT_TO_MINUTES(logging_tm);
  if (iNewLoggingTimeInMinutes < iOldLoggingTimeInMinutes) // Cater for 24 hour wrap case
    iElapsedLoggingTimeInMinutes = (TWENTYFOUR_HOURS_IN_MINUTES - iOldLoggingTimeInMinutes) + iNewLoggingTimeInMinutes;
  else
    iElapsedLoggingTimeInMinutes = iNewLoggingTimeInMinutes - iOldLoggingTimeInMinutes;
  
  if ((isOncloseLoggingFilebuttonPin) && (currentLoggingStatus == eLoggingActive))
  {
    tmpBuf[0] = '0';
    tmpBuf[1] = 0;
    if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) 
      MQTTclient.publish(getLoggingStatusConfirmTopic, tmpBuf);
    newLoggingStatus = eLoggingInactive;
    isOncloseLoggingFilebuttonPin = false;
    isOnopenLoggingFilebuttonPin = false;
  }
  
  if ((currentLoggingStatus == eLoggingActive) && 
      (newLoggingStatus == eLoggingActive) && 
      (iElapsedLoggingTimeInMinutes >= iLoggingPeriodInMinutes) &&
      (logFile))
  {
    // logging interval has elapsed
    old_logging_tm = logging_tm;
    
    sprintf(tmpLogFileStr,"%02d/%02d/%04d,%02d:%02d:%02d,",logging_tm.Day,logging_tm.Month,tmYearToCalendar(logging_tm.Year),logging_tm.Hour,logging_tm.Minute,logging_tm.Second);    
    #ifdef DEBUG_LOGGING
    Serial.print("Logged file : ");
    Serial.println(logFile);
    Serial.print("Logged : ");
    Serial.print(tmpLogFileStr);
    #endif
    yield();
    if (logFile.print(tmpLogFileStr) != 0)
    {
      sprintf(tmpLogFileStr,"%s,%s,%s,%s",strTempC.c_str(),strHumid.c_str(),strTempBaro,strAmbientLightLevel.c_str());
      #ifdef DEBUG_LOGGING
      Serial.println(tmpLogFileStr);
      #endif
      if (logFile.println(tmpLogFileStr) == 0)
      {
        newLoggingStatus = eLoggingFault;
      } 
    } else {
        newLoggingStatus = eLoggingFault;
    }
  }

  if ((isOnopenLoggingFilebuttonPin) && (currentLoggingStatus == eLoggingInactive))
  {
    tmpBuf[0] = '1';
    tmpBuf[1] = 0;
    if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE) 
      MQTTclient.publish(getLoggingStatusConfirmTopic, tmpBuf);
    newLoggingStatus = eLoggingInitialise;
    isOnopenLoggingFilebuttonPin = false;
    isOncloseLoggingFilebuttonPin = false;
  }
    
  if (currentLoggingStatus != newLoggingStatus)
  {
    switch (currentLoggingStatus)
    {
      case eLoggingInitialise :
                                switch (newLoggingStatus) {
                                  case eLoggingInitialise : // Do nothing, can't get here
                                  case eLoggingActive :     // Do nothing, can't get here
                                  case eLoggingInactive :   // Do nothing, can't get here
                                  case eLoggingFault :      // Do nothing, can't get here
                                  default : currentLoggingStatus = eLoggingInactive;
                                            newLoggingStatus     = eLoggingInactive;
                                            #ifdef DEBUG_LOGGING
                                            Serial.println(F("Error : CLS Initialise"));
                                            #endif
                                }
                                break;
      case eLoggingActive :
                                switch (newLoggingStatus) {
                                  case eLoggingInactive :   if (logFile) 
                                                              logFile.close();
                                                            currentLoggingStatus = eLoggingInactive;
                                                            #ifdef DEBUG_LOGGING
                                                            Serial.println(F("File Closed : CLS Active, NLS Inactive"));
                                                            #endif
                                                            logFilename[0] = 0;
                                                            shortLogFilename[0] = 0;
                                                            lcd.setCursor(0,0);
                                                            lcd.writeStr("NO LOGGING "); // print message
                                                            //lcd.writeStr("           "); 
                                                            break;
                                  case eLoggingFault :      if (logFile) // Captures case where device was logging and failed to write to file
                                                              logFile.close();
                                                            currentLoggingStatus = eLoggingInactive;
                                                            newLoggingStatus     = eLoggingInactive;
                                                            #ifdef DEBUG_LOGGING
                                                            Serial.println(F("File Closed : CLS Active, NLS Fault"));
                                                            #endif
                                                            logFilename[0] = 0;
                                                            shortLogFilename[0] = 0;
                                                            lcd.setCursor(0,0);
                                                            lcd.writeStr("NO LOGGING "); // print message
                                                            //lcd.writeStr("           "); 
                                                            break;
                                  case eLoggingInitialise :  // Do nothing, can't get here 
                                  case eLoggingActive     :  // Do nothing, can't get here
                                  default : newLoggingStatus = eLoggingActive;
                                            #ifdef DEBUG_LOGGING
                                            Serial.println(F("Error : CLS Active"));
                                            #endif
                                }
                                break;
      case eLoggingInactive :
                                switch (newLoggingStatus) {
                                  case eLoggingInitialise : //sprintf(logFilename,"%02d-%02d-%02d_%02d%02d%02d%s",logging_tm.Day,logging_tm.Month,tmYearToCalendar(logging_tm.Year),logging_tm.Hour,logging_tm.Minute,logging_tm.Second,fileNameTypeString);
                                                            sprintf(logFilename,"%02d%02d%02d%02d%s",logging_tm.Day,logging_tm.Month,logging_tm.Hour,logging_tm.Minute,fileNameTypeString);
                                                            sprintf(shortLogFilename,"%02d%02d%02d%02dcsv",logging_tm.Day,logging_tm.Month,logging_tm.Hour,logging_tm.Minute);
                                                            #ifdef DEBUG_LOGGING
                                                            Serial.print("File Opened : ");
                                                            Serial.println(logFilename);
                                                            #endif
                                                            logFile = SD.open(logFilename,SD_FILE_WRITE_MODE); // SQ TODO
                                                            yield();
                                                            //logFile = SD.open(logFilename,FILE_WRITE);
                                                            if (logFile) {
                                                              if (logFile.println(csvColumnTitles) == 0) {
                                                                logFile.close();
                                                                currentLoggingStatus = eLoggingInactive;
                                                                newLoggingStatus     = eLoggingInactive;
                                                                logFilename[0] = 0;
                                                                shortLogFilename[0] = 0;
                                                                #ifdef DEBUG_LOGGING
                                                                Serial.println(F("Failed to print to file"));
                                                                #endif
                                                              } else {
                                                                old_logging_tm = logging_tm;
                                                                currentLoggingStatus = eLoggingActive;
                                                                newLoggingStatus     = eLoggingActive;
                                                                #ifdef DEBUG_LOGGING
                                                                Serial.println(F("Initialise : CLS Active, NLS Active"));
                                                                #endif
                                                                lcd.setCursor(0,0);
                                                                lcd.writeStr("           "); 
                                                                lcd.setCursor(0,0);
                                                                lcd.writeStr(shortLogFilename); 
                                                              }
                                                              yield();
                                                            } else {
                                                              currentLoggingStatus = eLoggingInactive;
                                                              newLoggingStatus     = eLoggingInactive;
                                                              #ifdef DEBUG_LOGGING
                                                              Serial.println(F("Failed to open file"));
                                                              #endif
                                                            }
                                                            break;
                                                            
                                  case eLoggingInactive : // Do nothing, can't get here
                                  case eLoggingActive :   // Do nothing, can't get here
                                  case eLoggingFault :    // Do nothing, can't get here
                                  default : newLoggingStatus = eLoggingInactive;
                                            #ifdef DEBUG_LOGGING
                                            Serial.println(F("Error : CLS Inactive"));
                                            #endif
                                }
                                break;
      case eLoggingFault :
                                switch (newLoggingStatus) { 
                                  case eLoggingInitialise : // Do nothing, can't get here
                                  case eLoggingActive :     // Do nothing, can't get here
                                  case eLoggingInactive :   // Do nothing, can't get here
                                  case eLoggingFault :      // Do nothing, can't get here
                                  default : currentLoggingStatus = eLoggingInactive;
                                            newLoggingStatus     = eLoggingInactive;
                                            #ifdef DEBUG_LOGGING
                                            Serial.println(F("Error : CLS Fault"));
                                            #endif
                                }
                                break;
    }
    //currentLoggingStatus = newLoggingStatus;
  }
}



void readConfigurableButtonInput(void)
{
  String sButtonTopic;
  String sButtonPayload;
  bool bButAssigned;
  bool bOk = true;
  bool bDone = false;
  int index = 0;

  File f = SD.open(CONFIGURABLE_BUTTON_INPUT_FILE, SD_FILE_READ_MODE);
  if (!f) {
    #ifdef DEBUG_BUTTONS
    Serial.println("Failed to read Button Configurable Input Vals.");
    #endif
    bAllowConfigurableButtonControl = false;
    return;
  } else {
    #ifdef DEBUG_BUTTONS
    Serial.print("From : "); Serial.println(CONFIGURABLE_BUTTON_INPUT_FILE);
    #endif
    index = 1;
    bOk   = true;
    bDone = false;
    while ((f.position() < f.size()) && (bOk) && (!bDone))
    {
      if (f.position() < f.size()) {
        sButtonTopic=f.readStringUntil('\n'); sButtonTopic.trim();
      } else {
        bOk = false;
        bAllowConfigurableButtonControl = false;
      }
      if (f.position() < f.size()) {
        sButtonPayload=f.readStringUntil('\n'); sButtonPayload.trim();
      } else {
        bOk = false;
        bAllowConfigurableButtonControl = false;
      }
      bButAssigned = false;
      if (((sButtonTopic.length()   > 0) && (sButtonTopic.length()   <= (MAX_TOPIC_STRING + 1))) && (bOk) &&
          ((sButtonPayload.length() > 0) && (sButtonPayload.length() <= (MAX_MQTT_PAYLOAD_STRING + 1)))) {

            if ( (sButtonTopic.indexOf(strEmptyEntry) == -1) && (sButtonPayload.indexOf(strEmptyEntry) == -1)){
              strcpy(ConfigurableButtonInstanceArray[index-1].strButtonTopic,sButtonTopic.c_str());  
              strcpy(ConfigurableButtonInstanceArray[index-1].strButtonPayload,sButtonPayload.c_str());  
              ConfigurableButtonInstanceArray[index-1].bButtonAssigned = true;
              bButAssigned = true;
            } else {
              ConfigurableButtonInstanceArray[index-1].bButtonAssigned = false;
            }
            index++;

            if (index > MAX_CONFIGURABLE_BUTTON_INSTANCES) {
              bDone = true;
              bAllowConfigurableButtonControl = true;
            }
          
      } else {
          bOk = false;
          bAllowConfigurableButtonControl = false;
      }
      #ifdef DEBUG_BUTTONS
      Serial.println(sButtonTopic);
      Serial.println(sButtonPayload);
      Serial.println(bButAssigned);
      #endif
    } 
    f.close();
    yield();
  }
  
  #ifdef DEBUG_BUTTONS
  {
    for(int x = 0; x < MAX_CONFIGURABLE_BUTTON_INSTANCES; x++) {
      Serial.print("Entry #         : "); Serial.println(x);
      Serial.print("Topic Name      : "); Serial.println(ConfigurableButtonInstanceArray[x].strButtonTopic);
      Serial.print("Payload Value   : "); Serial.println(ConfigurableButtonInstanceArray[x].strButtonPayload);
      Serial.print("Button Assigned : "); Serial.println(ConfigurableButtonInstanceArray[x].bButtonAssigned);
    }
  }
  #endif
}


void printBarometricTrend(eBAROMETRIC_PRESSURE_STATE CurrentBarometricTrend) 
{
  if (bBMPSensorFail) return;
  lcd.setCursor(10,1);
  switch (CurrentBarometricTrend) {
    case eRISING  : lcd.write(BaroUp);
                    break;
    case eSTABLE  : lcd.writeStr("-");
                    break;
    case eFALLING : lcd.write(BaroDown);
                    break;
  }
}


