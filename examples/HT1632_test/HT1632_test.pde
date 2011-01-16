/***********************************************************************
 * HT1624.pde - Arduino demo program for Holtek HT1632 LED driver chip,
 *            As implemented on the Sure Electronics DE-DP016 display board
 *            (16*24 dot matrix LED module.)
 * Nov, 2008 by Bill Westfield ("WestfW")
 *   Copyrighted and distributed under the terms of the Berkely license
 *   (copy freely, but include this notice of original author.)
 *
 * Adapted for 8x32 display by FlorinC.
 ***********************************************************************/

// comment out this line for the 8x32 display;
//#define _16x24_

#include <wprogram.h>
#include <avr\pgmspace.h>
#include <HT1632_LedMatrix.h>

// create object to control the LED Matrix
HT1632_LedMatrix led = HT1632_LedMatrix();

#define DISPDELAY 60

char* msg = "        Christmas Market Friday 26th to Sunday 29th 10am to 4pm";
int crtPos = 0;
int msgx = 1;    // position on message screen of current character
                  // set to 1 to allow for first scroll
/*
* This works equally well for both 16x24 and 8x32 matrices.
*/
void displayScrollingLine()
{
  int y,xmax,ymax;
  led.getXYMax(&xmax,&ymax);
  // shift the whole screen 6 times, one column at a time;
  for (int x=0; x < 6; x++)
  {
    led.scrollLeft(1);
    msgx--;
    // fit as much as we can on
    /*
    while (!led.putChar(msgx,0,msg[crtPos]))  // zero return if it all fitted
    {
      led.getXY(&msgx,&y);
      crtPos++; // we got all of the character on!!
      if (crtPos >= strlen(msg))
      {
        crtPos = 0;
      }
    }
    */
    while (!led.putChar(msgx,0,crtPos+32))  // zero return if it all fitted
    {
      led.getXY(&msgx,&y);
      crtPos++; // we got all of the character on!!
      if (crtPos >= 92)
      {
        crtPos = 0;
      }
    }
   led.putShadowRam();
    delay(DISPDELAY);
  }

}

/***********************************************************************
 * traditional Arduino sketch functions: setup and loop.
 ***********************************************************************/

void setup ()
{
  led.init(1,1);
  Serial.begin(115200);
  led.clear();
  
  uint32_t time = millis();
  for (int i=0; i<1000; i++)
    led.clear();
  time = millis() - time;
  Serial.print("Time for 1000 clears is "); Serial.print(time); Serial.println(" mS");
  led.setBrightness(2);
}

void loop ()
{
  displayScrollingLine();
}

