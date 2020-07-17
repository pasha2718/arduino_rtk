/*
 * Configure and enable a ZED-F9P as a rover station
 * Receive RTCM messages on LoRa and relay them to Ublox ZED-F9P module
 * Display results on LCD
*/

#include <SPI.h>
#include <Wire.h> //Needed for I2C to GPS
#include <RH_RF95.h> // Radio Head Library:
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
  SetupRadio();     // set up the LoRa Radio
  InitRTCM();       // set up receipt of RTCM messages on radio

  delay(1000);      // just so I can see it before the party starts
}

void loop()
{
  if (millis() - LastPaint > 10000) {
    // only refresh screen every 10 seconds, it's slow and we are focusing on messages

    //Turn on Red LED if we haven't received a packet after 10s
    if(millis() - LastMsgTime > 10000) {
      DEBUG_PRINTLN("No message in 10s");
      digitalWrite(LED_PIN_RED, HIGH); // Turn on red LED

    } else {
      digitalWrite(LED_PIN_RED, LOW); // Turn off red LED

      digitalWrite(LED_PIN_BLUE, HIGH);  // turn on blue LED, indicating timely messages
      digitalWrite(LED_PIN_BLUE2, HIGH); // turn on blue LED, indicating timely messages
    }

    PaintScreen(RxCount-LastRxCount, RxByteCount-LastRxByteCount);

    LastRxCount = RxCount;
    LastRxByteCount = RxByteCount;
    LastPaint = millis();
  }

  // regardless of switch position, do the below
  GetRTCM();    // take bytes from LoRa, form into sentences, throw them away

  LastSwitch = SwitchState;

  // no need to delay because we are waiting for radio messages in GetRTCM()
  /* delay(100); */ 
}

/*
 * Set up LEDs, LCD, GPS, Radio here, just to keep setup() neat
*/
void SetupLED()
{
  DEBUG_BEGIN();
  DEBUG_PRINTLN("Booting Debug Listener");

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

void SetupRadio()
{
  // set up the LoRa radio
  LCD.setCursor(0, 3);
  LcdPad("Radio Init", 20);

  // Initialize the Radio. 
  if (RF95.init() == true) {
    DEBUG_PRINTLN(F("Radio is ready"));
    LCD.setCursor(0, 3);
    LcdPad("Radio is ready", 20);
  } else {
    DEBUG_PRINTLN(F("Radio Init Failed - Freezing"));
    LCD.setCursor(0, 3);
    LcdPad("Radio Init Failed", 20);
    digitalWrite(LED_PIN_RED, HIGH); // turn on red LED, indicating error
    while (1);
  }

  RF95.setFrequency(FREQUENCY); 
  RF95.setModemConfig(RH_RF95::Bw500Cr45Sf128);    // change config to fast+shortrange
}

void InitRTCM()
{
  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("Starting RTCM Reception"));

  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_BLUE, LOW);
  digitalWrite(LED_PIN_BLUE2, LOW);

  RxCount = 0;
  RxByteCount = 0;
  StartTime = millis();

  LCD.clear();
  PaintScreen(0, 0);

  // overcoming a bug where ROVER won't print first 10 secs
  LCD.setCursor(0, 0);
  LcdPad("LISTEN", 9);
}

/*
 * RelayRTCM() gets called each loop of rover mode
 survey seems to wipe some of it out
*/
void GetRTCM()
{
  uint8_t relayCount = 0;
   // get up to 10 messages in this batch, we need to bet back to loop()
  while (RF95.available() && relayCount++ < 10) { 

    uint8_t loraBuf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t loraLen = sizeof(loraBuf);

    if (RF95.recv(loraBuf, &loraLen)){
      RxCount++;
      RxByteCount += loraLen;

      /* DEBUG_PRINT("New message: "); */
      /* DEBUG_PRINT((LoraBuf[3]) << 4 | (LoraBuf[4] >> 4));   // message type */
      /* DEBUG_PRINT("/"); */
      /* DEBUG_PRINT(((LoraBuf[1] & 0x03) << 8 | LoraBuf[2]) + 6); // message length */
      /* DEBUG_PRINT(" "); */

      LastMsgTime = millis();    // Timestamp this packet
      delay(10);                 // give the bus a tiny break

    } else {
      DEBUG_PRINTLN("Receive failed");
    }
  }
}

/*
 * paint screen as such:
 *   0.2.4.6.8.0.2.4.6.8.
 * 0 RTCM Acc 123.123m
 * 1 Lat  -123.1234567 F0    end with fix type
 * 2 Long -123.1234567 R0    end with rtk type
 * 3 Tx 123/123456 Sat 12
 *
 * title[] goes into (0,0)
 * trx[] goes into (0,3) and is supposed to be 2 characters
 * diffCount is the number of messages passed since the last display
 * diffBytes is the number of bytes passed since the last display 
*/
void PaintScreen(unsigned long diffCount, unsigned long diffBytes)
{
  char msgbuf[21] = {0};

  // rewrite title in case the push-button diagnostics overwrote it
  LCD.setCursor(0, 0);
  strcat(msgbuf, title);
  strcat(msgbuf, " Acc");
  LcdPad(msgbuf, 20);                     // this says "$title Acc" padded to full row

  uint32_t accuracy = MY_GPS.getPositionAccuracy();
  LCD.setCursor(strlen(msgbuf)+1, 0);     // place it one space after Acc
  // this number occasionally flips to a strange value, DK why.
  // in this case just flash "Inf" instead of the huge number
  if (accuracy < 1000000) {
    LCD.print(accuracy/1000., 3);
  } else {
    LCD.print("1000 ");
  }
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

  LCD.setCursor(18, 1);
  LCD.print("F");
  //Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning, 5=Survey-in
  LCD.print(MY_GPS.getFixType());   

  /* int32_t longitude = MY_GPS.getHighResLongitude(); */
  long longitude = MY_GPS.getLongitude();
  LCD.setCursor(0, 2);
  LcdPad("Long ", 20);
  LCD.setCursor(5, 2);
  if (longitude >= 0) {
    LCD.print(" ");
  }
  LCD.print(longitude / 10e6, 7);
  LCD.setCursor(18, 2);
  LCD.print("R");
  //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
  LCD.print(MY_GPS.getCarrierSolutionType());   

  LCD.setCursor(0, 3);
  strcpy(msgbuf, trx);
  strcat(msgbuf, "           Sat");
  LcdPad(msgbuf, 20);
  LCD.setCursor(3, 3);
  LCD.print(diffCount);       // number of msg sent last 10 sec
  LCD.print("=");
  // convert number of bytes sent last 10 sec to kbps
  float kbps = diffBytes / 10. * 8 / 1024.;
  LCD.print(kbps, 1);
  LCD.print("kb");

  LCD.setCursor(17, 3);
  LCD.print(MY_GPS.getSIV(500)); //Returns number of sats used in fix
}
/*
 * Print msg to the LCD, truncating at or padding to msglen
*/
void LcdPad(char msg[], int msglen)
{
  // TODO allow this to accept F() macro inputs
  char lcdmsg[21] = {0};

  // don't allow overflow
  if (msglen > 20) {
    msglen = 20;
  }

  if (strlen(msg) > msglen) {
    strncpy(lcdmsg, msg, msglen);   // trim it

  } else {
    strcpy(lcdmsg, msg);
    for (int i = strlen(msg); i < msglen; i++) {
      lcdmsg[i] = ' ';
    }
  }
  lcdmsg[msglen + 1] = 0;
  /* lcd.print(strlen(lcdmsg)); */
  LCD.print(lcdmsg);
}

/*
 * Print msg to the lcd, truncating at or padding to msglen
*/
void LcdPad(int msgi, int msglen)
{
  char msg[] = {0};
  itoa(msgi, msg, 10); 
  LcdPad(msg, msglen);
}

/*
 * convert difference between times into timestamp and print it
*/
void LcdTimestamp(unsigned long currTime, unsigned long startTime)
{
  char timestamp[9];
  unsigned long duration = (currTime - startTime) / 1000;
  /* duration += 200*3600 + 13*60; // testing limits */
  byte hours = duration/3600;
  byte minutes = (duration - hours * 3600) / 60;
  byte seconds = duration - hours * 3600 - minutes * 60;

  if (hours > 0) {
    sprintf(timestamp, "%i:%02i:%02i", hours, minutes, seconds);
  } else {
    sprintf(timestamp, "%02i:%02i", minutes, seconds);
  }
  LCD.print(timestamp);
}



/*
 * Return the number of free bytes in memory, approximately
*/
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int FreeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
