/*
 * paint top 3 lines of screen as such:
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
void PaintPosition(char title[])
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
}

/*
 * Paint the bytes on the bottom line for Tx
 *   0.2.4.6.8.0.2.4.6.8.
 * 3 Tx 123/123456 
 *
 * trx[] goes into (0,3) and is supposed to be 2 characters
 * diffCount is the number of messages passed since the last display
 * diffBytes is the number of bytes passed since the last display 
 */
void PaintTRx(char trx[], unsigned long diffCount, unsigned long diffBytes)
{
  LCD.setCursor(0, 3);
  LcdPad(trx, 20);
  LCD.setCursor(3, 3);
  LCD.print(diffCount);       // number of msg sent last 10 sec
  LCD.print("=");
  // convert number of bytes sent last 10 sec to KB/s kilobytes per second
  float kbps = diffBytes / 10. / 1000.;
  LCD.print(kbps, 1);
  LCD.print("kB");
}

/*
 * Paint the sat count on the bottom line
 *   0.2.4.6.8.0.2.4.6.8.
 * 3               Sat 12
 *
 * This assumes the line is cleared above
 */
void PaintSat()
{
  LCD.setCursor(13, 3);
  LCD.print("Sat ");
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
