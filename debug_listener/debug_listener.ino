/*
 * Just listen for RTCM messages on LoRa
 * This is to monitor that the gps_base is still running, for debugging purposes
 * Display timers, error message on LCD
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

    PaintScreen();

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
 * 0 RUNNING or TIMEOUT
 * 1 Uptime:  00:00:00
 * 2 Elapsed: 00:00:00
 * 3 Rx 123/123456              messages/bytes sent in last 10 sec
*/
void PaintScreen()
{
  LCD.setCursor(0, 0);

  if(millis() - LastMsgTime > 10000) {
    // we are in timeout mode
    LcdPad("TIMEOUT", 20);
  } else {
    // we are in normal good mode
    LcdPad("LISTEN", 20);
  }

  LCD.setCursor(0, 1);
  LCD.print("Elapsed:    ");
  LcdTimestamp(millis(), StartTime);

  LCD.setCursor(0, 2);
  LCD.print("Since Last: ");
  LcdTimestamp(millis(), LastMsgTime);

  LCD.setCursor(0, 3);
  LcdPad("Rx", 20);
  LCD.setCursor(3, 3);
  LCD.print(RxCount-LastRxCount);       // number of msg sent last 10 sec
  LCD.print("=");
  // convert number of bytes sent last 10 sec to kbps
  float kbps = (RxByteCount-LastRxByteCount) / 10. * 8 / 1024.;
  LCD.print(kbps, 1);
  LCD.print("kb");
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
