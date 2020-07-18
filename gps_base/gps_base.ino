/*
 * Configure and enable a ZED-F9P as a base station
 * Survey-in to position for 5-30 minutes AND 5 meters
 * Display survey results on LCD
 *
 * Then output RTCM from ZED-F9P to LoRa for the Rover
 * Configure Ublox ZED-F9P to send five RTCM messages
 * Relay RTCM bytes to radio packet broadcast for rover(s)
 * Display results on LCD
 * During RTCM mode, the button will show some diagnostics on LCD
*/

#include <SPI.h>
#include <Wire.h> //Needed for I2C to GPS
#include <RH_RF95.h> // Radio Head Library:
#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD

/* #define DEBUG */
#ifdef DEBUG
#define DEBUG_BEGIN()  SerialUSB.begin(9600); while (! SerialUSB); 
#define DEBUG_PRINT(x)  SerialUSB.print(x);
#define DEBUG_PRINT2(x,y)  SerialUSB.print(x,y);
#define DEBUG_PRINTLN(x)  SerialUSB.println(x);
#else
#define DEBUG_BEGIN()
#define DEBUG_PRINT(x)
#define DEBUG_PRINT2(x,y)
#define DEBUG_PRINTLN(x)
#endif

const int LED_PIN_RED =  9;       // Pin that has the red LED (shows error)
const int LED_PIN_BLUE = 4;       // Pin that has the blue LED (shows RTK1)
const int LED_PIN_BLUE2 = 3;      // Pin that has the second blue LED (shows RTK2)
const int SWITCH_PIN = 2;         // Pin that has the switch (shows diag)

// The broadcast frequency can be anywhere from 902-928MHz
const float FREQUENCY = 921.2;    // frequency in Mhz for broadcast

// Give the RFM95 module's chip select and interrupt pins to the RF95 instance
// On the SparkFun ProRF those pins are 12 and 6 respectively.
RH_RF95 RF95(12, 6);
SerLCD LCD; // Initialize the LCD library with default I2C address 0x72
SFE_UBLOX_GPS MY_GPS; // Initialize the GPS library with default I2C address 0x42

int SwitchState = HIGH;                // is the switch on or off now
int LastSwitch = LOW;                  // the SwitchState in the last loop
int LastPosPaint = 0;                  // whether we have painted the position
unsigned long TxCount = 0;             // how many RTCM messages have been sent on LoRa
unsigned long TxByteCount = 0;         // how many RTCM bytes have been sent on LoRa
unsigned long LastTxCount = 0;         // prior value for timing on LCD
unsigned long LastTxByteCount = 0;     // prior value for timing on LCD
unsigned long StartTime = 0;           // what time did we start the survey or RTCM in seconds
unsigned long LastPaint = 0;           // time of the last screen paint

uint16_t RTCMFrameCounter = 0;        // counter for bytes in RTCM message
uint16_t RTCMLen = 0;                 // length of RTCM message from length bits
uint16_t RTCMMsgType = 0;             // RTCM Message Type (1074, 1124, 1230, etc)
// the standard calls for 24+1023+24 = 1071 bytes, but in reality they tend smaller
// some go bigger than 512 though. So use full buffer
uint8_t RTCMSendBuffer[1071] = {0};   // buffer to assemble outgoing RTCM message

void setup()
{
  SetupLED();       // set up the LEDs
  SetupLCD();       // set up the SerLCD display
  SetupGPS();       // set up the zed-f9p RTK2 GPS
  SetupRadio();     // set up the LoRa Radio
  delay(1000);      // just so I can see it before the party starts

  WaitForFix();     // 
  RunSurvey();      // run the survey before starting RTCM loop
  InitRTCM();       // prepare to send RTCM to the RTK2
}

void loop()
{
  digitalWrite(LED_PIN_BLUE, HIGH);  // flash blue LED while painting

  SwitchState = digitalRead(SWITCH_PIN);      // chech switch/button status

  // the position doesn't change after survey so don't waste time re-accessing and painting it.
  if ( ! LastPosPaint ) {
    PaintPosition("RTCM");
    LastPosPaint = 1;
  }

  if (SwitchState == HIGH) {  // switch is off - do RTCM
    if (SwitchState != LastSwitch || millis() - LastPaint > 10000) {
      // only refresh screen every 10 seconds, it's slow and we are focusing on messages
      digitalWrite(LED_PIN_BLUE2, HIGH);  // flash blue LED while painting
      /* PaintScreen("RTCM", "Tx", TxCount-LastTxCount, TxByteCount-LastTxByteCount); */
      PaintTRx("Tx", TxCount-LastTxCount, TxByteCount-LastTxByteCount);
      digitalWrite(LED_PIN_BLUE2, LOW);  // flash blue LED while painting
      LastTxCount = TxCount;
      LastTxByteCount = TxByteCount;
      LastPaint = millis();
    }
  } else if (SwitchState != LastSwitch) { // switch is on - do diagnostics
    DoDiagnostics();                      // just run it once
    LastPosPaint = 0;                     // leave that first run on the screen until release
  }

  digitalWrite(LED_PIN_BLUE, LOW);  // flash blue LED while painting

  MY_GPS.checkUblox();     //See if new GPS data are available - sends bytes to processRTCM()

  LastSwitch = SwitchState;

  delay(250); //Don't pound too hard on the I2C bus
}

/*
 * Set up LEDs, LCD, GPS, Radio here, just to keep setup() neat
*/
void SetupLED()
{
  DEBUG_BEGIN();
  DEBUG_PRINTLN("Booting GPS Base");

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
  LcdPad("LCD Ready", 20);
}

void SetupGPS()
{
  MY_GPS.begin(Wire);

  if (MY_GPS.isConnected() == false) {
    DEBUG_PRINTLN(F("Ublox GPS not detected. Freezing."));
    LCD.setCursor(0, 1);
    LcdPad("No GPS detected", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1);
  }

  // This clock speed is good for the LCD. GPS integ pg 11 says 400khz is max speed
  // The lora radio doesn't use i2c so no problem there.
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  LCD.setCursor(0, 1);
  LcdPad("GPS Detected", 20);

  // Example code looks for an old running survey and reconnects.
  // This causes problems when you move sites, it just skips the survey and starts at the wrong place.
  // Instead, let's do a full reset to clear the field
  LCD.setCursor(0, 1);
  LcdPad("GPS Factory Reset", 20);
  MY_GPS.factoryReset(); // factory reset to get rid of prior positions, surveys, everything

  // more config is in InitRTCM() - survey seems to wipe some of it out
}

void SetupRadio()
{
  // set up the LoRa radio
  LCD.setCursor(0, 3);
  LcdPad("Radio Init", 20);

  // Initialize the Radio. 
  if (RF95.init() == true) {
    DEBUG_PRINTLN(F("Radio is ready"));
    LCD.setCursor(0, 2);
    LcdPad("Radio ready", 20);
  } else {
    DEBUG_PRINTLN(F("Radio Init Failed - Freezing"));
    LCD.setCursor(0, 2);
    LcdPad("Radio Init Failed", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1);
  }

  RF95.setFrequency(FREQUENCY); 

  if (RF95.isChannelActive()) {
    LCD.setCursor(0, 2);
    LcdPad("Radio Freq in Use", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1);
  }

  RF95.setModemConfig(RH_RF95::Bw500Cr45Sf128);    // change config to fast+shortrange
  RF95.setTxPower(20, false);     // boost power to 20db, default 13
}

/*
 * WaitForFix() waits for a 3D fix
 * It looks like RunSurvey is NOT happy until there is a fix
 * and we lose our fix with a reset.  So let's wait for it.
*/
void WaitForFix()
{
  unsigned long startFix = millis();
  DEBUG_PRINTLN(F("Waiting for fix before survey-in..."));
  LCD.setCursor(0, 3);
  LcdPad("Wait fix", 20);

  while (MY_GPS.getFixType() < 3) {

    LCD.setCursor(9, 3);
    LcdTimestamp(millis(), startFix);
    delay(1000);
  }
}

/*
 * RunSurvey() gets at the end of loop() to do the survey-in-place
 * Run a 30-minute survey by default,
 * but if you start it with the button down, it will go to 5 minute
*/
void RunSurvey()
{
  uint8_t surveyMinutes = 30;
  // if the button is down, survey 5 minutes instead of 5
  if (digitalRead(SWITCH_PIN) == LOW) {
    surveyMinutes = 5;
  }

  digitalWrite(LED_PIN_BLUE, HIGH);  // turn on both blue LEDs, indicating survey
  digitalWrite(LED_PIN_BLUE2, HIGH); 

  LCD.clear();
  LCD.setCursor(0, 0);
  LcdPad("SURVEY Prep", 20);

  bool response = MY_GPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
  if (response == false) {
    DEBUG_PRINTLN(F("Failed to get Survey-In status. Freezing."));
    LCD.setCursor(0, 2);
    LcdPad("Svin query failed", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1) ;
  }

  if (MY_GPS.svin.active == true) {
    DEBUG_PRINTLN(F("Stopping prior survey"));
    LCD.setCursor(0, 2);
    LcdPad("Stop prior survey", 20);
    MY_GPS.disableSurveyMode(); // Disable a running survey
  }

  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("Starting new survey"));
  LCD.setCursor(0, 2);
  LcdPad("Starting new survey", 20);

  response = MY_GPS.enableSurveyMode(surveyMinutes * 60, 5.000); // start Survey-In, 5/30 minutes, 5.0m

  if (response == false) {
    DEBUG_PRINTLN(F("Survey start failed. Freezing."));
    LCD.setCursor(0, 2);
    LcdPad("Survey start failed", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1) ;
  }

  StartTime = MY_GPS.svin.observationTime;

  DEBUG_PRINTLN(F("Survey started. This will run until minutes has passed AND less than 5m accuracy is achieved."));

  LCD.clear();
  LCD.setCursor(0, 0);
  LcdPad("SURVEY", 20);

  //Begin waiting for survey to complete
  while (MY_GPS.svin.valid == false) {
    response = MY_GPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)

    if (response == true) {
      /* 
       * paint screen as such:
       *   0.2.4.6.8.0.2.4.6.8.
       * 0 SURV12 Acc 123.123m
       * 1 Lat  1234.1234567     <- overwrite with init/error text
       * 2 Long 1234.1234567     <- overwrite with init/error text
       * 3 T 123:12:12 Sat 12
      */
      char msgbuf[21] = {0};

      // rewrite title in case the push-button diagnostics overwrote it
      LCD.setCursor(0, 0);
      LcdPad("SURVEY Acc", 20);

      // use the mean survey-in accuracy, in meters
      // not MY_GPS.getPositionAccuracy() or not uint32_t accuracy = MY_GPS.getHorizontalAccuracy();
      float accuracy = MY_GPS.svin.meanAccuracy;
      /* float accuracy = MY_GPS.getPositionAccuracy(); */
      LCD.setCursor(11, 0);
      LCD.print(accuracy, 3);
      LCD.print("m");

      /* int32_t latitude = MY_GPS.getHighResLatitude(); */
      long latitude = MY_GPS.getLatitude();
      LCD.setCursor(0, 1);
      LcdPad("Lat ", 20);
      LCD.setCursor(5, 1);
      if (latitude >= 0) {
        LCD.print(" ");
      }
      LCD.print(latitude / 10e6, 7);

      /* int32_t longitude = MY_GPS.getHighResLongitude(); */
      long longitude = MY_GPS.getLongitude();
      LCD.setCursor(0, 2);
      LcdPad("Long ", 20);
      LCD.setCursor(5, 2);
      if (longitude >= 0) {
        LCD.print(" ");
      }
      LCD.print(longitude / 10e6, 7);

      LCD.setCursor(0, 3);
      LcdPad("T            Sat", 20);
      LCD.setCursor(1, 3);
      LCD.print(surveyMinutes);
      LCD.setCursor(4, 3);
      LcdTimestamp(millis(), StartTime);

      LCD.setCursor(17, 3);
      LCD.print(MY_GPS.getSIV(500)); //Returns number of sats used in fix

    } else {
      LCD.setCursor(0, 0);
      LcdPad("SURVEY Request Fail", 20);
    }
    delay(500);
  }           // while svin.valid = false

  digitalWrite(LED_PIN_BLUE, LOW);  // turn off blue LEDs, indicating survey is done
  digitalWrite(LED_PIN_BLUE2, LOW);
  DEBUG_PRINTLN(F("Base survey complete!"));
  LCD.setCursor(0, 3);
  LcdPad("SURVEY Complete!", 20);
}

/*
 * InitRTCM() gets called when entering RTCM mode anew
*/
void InitRTCM()
{
  // if you do it before the survey you may not keep the values.  better to set it now.
  //
  LCD.clear();
  LCD.setCursor(0, 0);
  LcdPad("RTCM Setup", 20);

  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_BLUE, LOW);

  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("Final config for RTCM"));

  bool response = true;

  // These are in fact the messages that are suggested in the integ doc page 19
  // We are sending them in UBX format not NMEA, once a second except 1230
  response &= MY_GPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); // "Stationary RTK Reference station ARP"
  // MSM4 messages here 
  response &= MY_GPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1); // GPS MSM4
  response &= MY_GPS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1); // GLONASS MSM4
  response &= MY_GPS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1); // Galileo MSM4
  /* response &= MY_GPS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1); // BeiDou MSM4 */
  response &= MY_GPS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); // GLONASS code-phase biases
  // that's it for RTCM

  // not saving config - caused problems and we set it anew each time
  /* MY_GPS.saveConfiguration();          //Save the current settings to flash and BBR */

  if (response == true) {
    DEBUG_PRINTLN(F("RTCM messages enabled"));
    LCD.setCursor(0, 1);
    LcdPad("GPS RTCM Enabled", 20);
  } else {
    DEBUG_PRINTLN(F("RTCM failed to enable. Are you sure you have an ZED-F9P? Freezing."));
    LCD.setCursor(0, 1);
    LcdPad("GPS RTCM Fail Config", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1); //Freeze
  }

  // Dynamic model tells the GPS what kind of movement to expect.  We're static for base.
  // We would be DYN_MODEL_STATIONARY for base
  response &= MY_GPS.setDynamicModel(DYN_MODEL_STATIONARY);

  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("Starting RTCM Transmissions"));
  LCD.setCursor(0, 2);
  LcdPad("Starting RTCM", 20);

  // Set the RTK2 chip to output UBX and RTCM sentences to the I2C port
  // with that, runnning checkUblox() in RunRTCM() will send RTCM bytes to processRTCM() below
  MY_GPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);

  TxCount = 0;
  TxByteCount = 0;
  StartTime = millis();
}

/*
 * Assuming we have configured to allow RTCM3 messages in.
 * Each loop() calls RunRTCM() which calls checkUblox() in the library.
 * That calls process() which has a hook to this processRTCM() below for each RTCM byte.
 * We parse them into sentences and send them on as radio packets.
 * Sending a stream of bytes didn't work and it feels like would have a lot of header overhead
*/
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  /*
   * we get the RTCM messages byte by byte, beginning with 0xD3
   * They are variable length and the meaning is in the bits.
   * To parse the bits and bytes into sentences, we copied logic from the library.
   * We copied logic from SFE_UBLOX_GPS::processRTCMframe(uint8_t incoming)
   * which processes every byte, then sends it to this function.
   * We want to do the same, forming sentences and sending packets to the radio
  */

  /* digitalWrite(LED_PIN_BLUE, HIGH);  // flash blue LED1 while in this loop (trying to catch where it sticks) */

  // note that RTCMFrameCounter is the *prior* count
  // TODO C consider removing this msgtype business if things get slow, it's just debug
  if (RTCMFrameCounter == 1) {
    RTCMLen = (incoming & 0x03) << 8; //Get the last two bits of this byte. Bits 8&9 of 10-bit length
  } else if (RTCMFrameCounter == 2) {
    RTCMLen |= incoming; //Bits 0-7 of packet length
    RTCMLen += 6;        //There are 6 additional bytes of what we presume is header, msgType, CRC
  } else if (RTCMFrameCounter == 3) {
    RTCMMsgType = incoming << 4; //Message Type, MS 4 bits
  } else if (RTCMFrameCounter == 4) {
    RTCMMsgType |= (incoming >> 4); //Message Type, bits 0-7
  }

  RTCMFrameCounter++;     // number of bytes in this message so far

  if (RTCMFrameCounter <= sizeof(RTCMSendBuffer)) {
    RTCMSendBuffer[RTCMFrameCounter-1] = incoming; // add this to the buffer to send
  } else {
    DEBUG_PRINTLN(F("RTCM Message too big - freezing"));
    LcdPad("RTCM msg too big", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while(1);
  }

  if (RTCMFrameCounter > 4 && RTCMFrameCounter == RTCMLen) { // we have a complete message

    /* if (TxCount % 8 == 0) { */
    /*   DEBUG_PRINTLN(); */
    /* } */
    /* DEBUG_PRINT(RTCMMsgType); */
    /* DEBUG_PRINT("="); */
    /* DEBUG_PRINT(RTCMLen); */
    /* DEBUG_PRINT(" "); */

    /* DEBUG_PRINT(F(".")); */

    /* digitalWrite(LED_PIN_BLUE2, HIGH);  // flash blue LED2 while transmitting */

    // send the full message out on the radio
    RF95.send(RTCMSendBuffer, RTCMLen);
    RF95.waitPacketSent();

    /* digitalWrite(LED_PIN_BLUE2, LOW);   // end of BLUE2 flash */

    TxCount++;
    TxByteCount += RTCMLen;

    // clear the counters
    RTCMFrameCounter = 0;
    RTCMLen = 0;
    RTCMMsgType = 0;
    // don't need to clear the buffer because counter 
    /* memset(RTCMSendBuffer, 0, sizeof(RTCMSendBuffer)); */

    /* digitalWrite(LED_PIN_BLUE, LOW);   // end of BLUE flash */

    // slight delay trying to keep it from freezing up
    delay(50);
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
  LCD.print("Chan Active: ");
  LCD.print(RF95.isChannelActive() ? "yes" : "no");
  /* LCD.print("RSSI "); */
  /* LCD.print(RF95.lastRssi(), DEC); */

  // Time
  LCD.setCursor(0, 3);
  LCD.print("Time ");
  LcdTimestamp(millis(), StartTime);

  // voltage?
}
