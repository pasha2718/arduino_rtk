/*
 * Configure and enable a ZED-F9P as a rover station
 * Receive RTCM messages on LoRa and relay them to Ublox ZED-F9P module
 * Display results on LCD
*/

#include <SPI.h>                            // Needed for access to radio
#include <Wire.h>                           // Needed for I2C to GPS and LCD
#include <RH_RF95.h>                        // Radio Head Library:
#include <SparkFun_Ublox_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS
#include <SerLCD.h>                         // Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
#include "git-version.h"

const int LED_PIN_RED =  9;       // Pin that has the red LED (shows error)
const int LED_PIN_BLUE = 4;       // Pin that has the blue LED (shows RTK1)
const int LED_PIN_BLUE2 = 3;      // Pin that has the second blue LED (shows RTK2)
const int SWITCH_PIN = 2;         // Pin that has the switch (shows diag)
const uint8_t GPS_I2C_ADDRESS = 0x42;

// The broadcast frequency can be anywhere from 902-928MHz
const float FREQUENCY = 921.2;    // frequency in Mhz for broadcast

// Give the RFM95 module's chip select and interrupt pins to the RF95 instance
// On the SparkFun ProRF those pins are 12 and 6 respectively.
RH_RF95 RF95(12, 6);
SerLCD LCD; // Initialize the LCD library with default I2C address 0x72
SFE_UBLOX_GPS MY_GPS; // Initialize the GPS library with default I2C address 0x42

int SwitchState = 0;              // is the switch on or off now
int LastSwitch = -1;              // the SwitchState in the last loop
unsigned long RxCount = 0;             // how many RTCM messages have been received on LoRa
unsigned long RxByteCount = 0;         // how many RTCM bytes have been rceived on LoRa
unsigned long LastRxCount = 0;         // prior value for timing on LCD
unsigned long LastRxByteCount = 0;     // prior value for timing on LCD
unsigned long StartTime = 0;           // what time did we start the rover in seconds
unsigned long LastPaint = 0;           // time of the last screen paint
unsigned long LastMsgTime = 0;             // time last message received from LoRa

void setup()
{
  SetupLED();       // set up the LEDs
  SetupLCD();       // set up the SerLCD display
  SetupGPS();       // set up the zed-f9p RTK2 GPS
  SetupRadio();     // set up the LoRa Radio
  delay(1000);      // just so I can see it before the party starts

  InitRTCM();       // prepare to receive RTCM on LoRa, send to ZED
}

void loop()
{
  SwitchState = digitalRead(SWITCH_PIN);      // chech switch/button status for which screen to show

  if (SwitchState == HIGH) {    // switch is off - show RTCM screen every 10s

    if (millis() - LastPaint > 10000) {
      // only refresh screen every 10 seconds, it's slow and we are focusing on messages
      PaintPosition("ROVER");
      PaintTRx("Rx", RxCount-LastRxCount, RxByteCount-LastRxByteCount);

      //Turn on Red LED if we haven't received a packet after 10s
      if(millis() - LastMsgTime > 10000) {
        digitalWrite(LED_PIN_RED, HIGH); // Turn on red LED
      } else {
        digitalWrite(LED_PIN_RED, LOW); // Turn off red LED
      }

      // turn on Blue LEDs if RTK
      switch (MY_GPS.getCarrierSolutionType()) {   //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
        case 0:
          digitalWrite(LED_PIN_BLUE, LOW);  // turn off blue LED, indicating no RTK float
          digitalWrite(LED_PIN_BLUE2, LOW); // turn off blue LED, indicating no RTK fixed
          break;
        case 1:
          digitalWrite(LED_PIN_BLUE, HIGH);  // turn on blue LED, indicating RTK float
          digitalWrite(LED_PIN_BLUE2, LOW);  // turn off blue LED, indicating no RTK fixed
          break;
        case 2:
          digitalWrite(LED_PIN_BLUE, HIGH);  // turn on blue LED, indicating RTK float
          digitalWrite(LED_PIN_BLUE2, HIGH); // turn on blue LED, indicating RTK fixed
          break;
      }

      LastRxCount = RxCount;
      LastRxByteCount = RxByteCount;
      LastPaint = millis();
    }

  } else if (SwitchState != LastSwitch) { // switch is on - show diagnostics
    DoDiagnostics();        // just run it once
  } // Switchstate

  // regardless of switch position, do the below
  RelayRTCM();    // take bytes from LoRa, form into sentences, relay to ZED

  LastSwitch = SwitchState;

  // no need to delay because we are waiting for radio messages in RelayRTCM()
  /* delay(100); */ 
}

/*
 * Set up LEDs, LCD, GPS, Radio here, just to keep setup() neat
*/
void SetupLED()
{
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);
  pinMode(LED_PIN_BLUE2, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // flash both leds on startup
  digitalWrite(LED_PIN_RED, HIGH);
  digitalWrite(LED_PIN_BLUE, HIGH);
  digitalWrite(LED_PIN_BLUE2, HIGH);
  delay(100);
  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_BLUE, LOW);
  digitalWrite(LED_PIN_BLUE2, LOW);
}

void SetupLCD()
{
  Wire.begin();
  LCD.begin(Wire);            //Set up the LCD for Serial communication at 9600bps

  LCD.setBacklight(0, 0, 0);    // black is off
  /* LCD.setBacklight(0x808080);    // grey */

  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("GPS Rover");
  LCD.setCursor(0, 1);
  LcdPad(GIT_VERSION, 20);

  LCD.setCursor(0, 2);
  LCD.print("LCD ");
}

void SetupGPS()
{
  MY_GPS.begin(Wire);

  if (MY_GPS.isConnected() == false) {
    LCD.setCursor(0, 3);
    LcdPad("No GPS detected", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1);
  }

  // This clock speed is good for the LCD. GPS integ pg 11 says 400khz is max speed
  // The lora radio doesn't use i2c so no problem there.
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  // Example code looks for an old running survey and reconnects.
  // This causes problems when you move sites, it just skips the survey and starts at the wrong place.
  // Instead, let's do a full reset to clear the field
  MY_GPS.factoryReset(); // factory reset to get rid of prior positions, surveys, everything

  LCD.print("GPS ");

  bool response = true;
  // we want UBX and NMEA to come out of the USB port for the laptop user.
  response &= MY_GPS.setUSBOutput(COM_TYPE_UBX);
  /* response &= MY_GPS.setUSBOutput(COM_TYPE_UBX | COM_TYPE_NMEA); */
  // configure the I2C port to expect input of RTCM3 messages, they are coming from LoRa to I2C
  response &= MY_GPS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_RTCM3, 250); 

  // now let's set up the UBX we want to see on USB
  response = true;
  response &= MY_GPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_SIG, COM_PORT_USB, 1);
  response &= MY_GPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_USB, 1);
  response &= MY_GPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_POSLLH, COM_PORT_USB, 1);
  response &= MY_GPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, COM_PORT_USB, 1);
  response &= MY_GPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_USB, 1);

  // These are the NMEA sentences that GPSD uses, and that we need
	/* response &= MY_GPS.enableNMEAMessage(UBX_NMEA_GBS, COM_PORT_USB);   // satt fault detection */
	response &= MY_GPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_USB);   // global position sys
	response &= MY_GPS.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_USB);   // geog position
	response &= MY_GPS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_USB);   // gnss dop and active satt
	/* response &= MY_GPS.enableNMEAMessage(UBX_NMEA_GST, COM_PORT_USB);   // pseudorange error */
	response &= MY_GPS.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_USB);   // satt in view
	response &= MY_GPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_USB);   // recommended min gnss data
	/* response &= MY_GPS.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_USB);   // course over ground */
	/* response &= MY_GPS.enableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_USB);   // time and date */

  if (response == true) {
    LCD.print("NMEA ");
  } else {
    LCD.setCursor(0, 3);
    LcdPad("GPS UBX Fail Config", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1); //Freeze
  }

  // Dynamic model tells the GPS what kind of movement to expect.  We're walking or standing.
  // We would be DYN_MODEL_STATIONARY or DYN_MODEL_PEDESTRIAN
  response &= MY_GPS.setDynamicModel(DYN_MODEL_PEDESTRIAN);

  if (response == true) {
    LCD.print("RTCM ");
  } else {
    LCD.setCursor(0, 3);
    LcdPad("GPS Failed Configure", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1); //Freeze
  }
}

void SetupRadio()
{
  // Set up the LoRa radio
  // Initialize the Radio. 
  if (RF95.init() != true) {
    LCD.setCursor(0, 3);
    LcdPad("Radio Init Failed", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1);
  }

  RF95.setFrequency(FREQUENCY); 
  RF95.setModemConfig(RH_RF95::Bw500Cr45Sf128);    // change config to fast+shortrange

  LCD.print("Radio");
}

void InitRTCM()
{
  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_BLUE, LOW);
  digitalWrite(LED_PIN_BLUE2, LOW);

  RxCount = 0;
  RxByteCount = 0;
  StartTime = millis();

  LCD.clear();
  PaintPosition("ROVER");
  // overcoming a bug where ROVER won't print first 10 secs
  LCD.setCursor(0, 0);
  LcdPad("ROVER Acc", 9);
}

/*
 * RelayRTCM() gets called each loop of rover mode
 survey seems to wipe some of it out
*/
void RelayRTCM()
{
  uint8_t relayCount = 0;
   // get up to 10 messages in this batch, we need to bet back to loop()
  while (RF95.available() && relayCount++ < 10) { 

    uint8_t loraBuf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t loraLen = sizeof(loraBuf);

    if (RF95.recv(loraBuf, &loraLen)){
      RxCount++;
      RxByteCount += loraLen;

      // Send this RTCM packet via I2C to RTK2
      Wire.beginTransmission(GPS_I2C_ADDRESS);
      for (int i=0; i<loraLen; i++) {
        Wire.write(loraBuf[i]);  // data bytes are queued in local buffer
      }
      Wire.endTransmission();

      LastMsgTime = millis();    // Timestamp this packet
      delay(10);                 // give the bus a tiny break
  }
}

void DoDiagnostics()
{
  LCD.clear();
  LCD.setCursor(0, 0);
  LcdPad("DIAGNOSTICS", 20); 

  LCD.setCursor(0, 1);
  LCD.print("Memory ");
  LCD.print(FreeMemory());

  // look at interference on server, RSSI on client
  LCD.setCursor(0, 2);
  /* LCD.print("Chan Active: "); */
  /* LCD.print(RF95.isChannelActive() ? "yes" : "no"); */
  LCD.print("RSSI ");
  LCD.print(RF95.lastRssi(), DEC);

  // Time
  LCD.setCursor(0, 3);
  LCD.print("Time ");
  LcdTimestamp(millis(), StartTime);

  // TODO other possible diags: voltage, RF95.frequencyError(), RF95.lastSNR()
}
