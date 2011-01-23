/***********************************************************************
 * HT1632_LedMatrix.cpp - Arduino library for Holtek HT1632 LED driver chip,
 *     As implemented on the Sure Electronics DE-DP10X display board
 *       (8 x 32 dot matrix LED module.)
 *
 * Original code by:
 * Nov, 2008 by Bill Westfield ("WestfW")
 *   Copyrighted and distributed under the terms of the Berkely license
 *   (copy freely, but include this notice of original author.)
 *
 * Adapted for 8x32 display by FlorinC.
 *
 * Library Created and updated by Andrew Lindsay October/November 2009
 ***********************************************************************/

#include "HT1632_LedMatrix.h"
#include <inttypes.h>
#include <wprogram.h>
#include <avr/pgmspace.h>
#include "font_5x7_p.hpp"

// Use this define to select Direct port writes for speed or regular arduino digitalWrite functions
#undef DIRECTIO

/*
 * Set these constants to the values of the pins connected to the SureElectronics Module
 * NOTE - these are different from the original demo code by westfw
 *
 * Use Pin Mappings 8-11 for CS1 to 4, 12 for Data and 13 for Clock
 * Use mixture of #define and variables.
 * Pin numbers are for setup and port identifiers are for direct port writes.
 */


#define LOWPINS
//#define LOWPINS1
//#define HIGHPINS

#ifdef LOWPINS
// Port D defines for 2-7
// If using Nuelectronics ethernet shield, need to cut unused interrupt track to PD2
byte ht1632_cs[4] = {2, 3, 4, 5};    // Chip Select (1, 2, 3, or 4)
volatile uint8_t* ht1632_cs_Port[4] = {&PORTD, &PORTD, &PORTD, &PORTD};    // Chip Select (1, 2, 3, or 4)
volatile uint8_t* ht1632_cs_DDR[4] = {&DDRD, &DDRD, &DDRD, &DDRD};    // Chip Select (1, 2, 3, or 4)
#define HT1632_DATA       6      // Data pin
#define HT1632_DATA_PORT  PORTD  // Data port
#define HT1632_DATA_DDR   DDRD   // Data port
#define HT1632_WRCLK      7      // Write clock pin
#define HT1632_WRCLK_PORT PORTD  // Write clock port
#define HT1632_WRCLK_DDR  DDRD   // Write clock port
#endif

#ifdef LOWPINS1
// Port D defines for 2-7 - these pins follow the ribbon cable - makes building a connector easy
// If using Nuelectronics ethernet shield, need to cut unused interrupt track to PD2
byte ht1632_cs[4] = {4, 2, 3, 5};    // Chip Select (1, 2, 3, or 4)
volatile uint8_t* ht1632_cs_Port[4] = {&PORTD, &PORTD, &PORTD, &PORTD};    // Chip Select (1, 2, 3, or 4)
volatile uint8_t* ht1632_cs_DDR[4] = {&DDRD, &DDRD, &DDRD, &DDRD};    // Chip Select (1, 2, 3, or 4)
#define HT1632_DATA       7      // Data pin
#define HT1632_DATA_PORT  PORTD  // Data port
#define HT1632_DATA_DDR   DDRD   // Data port
#define HT1632_WRCLK      6      // Write clock pin
#define HT1632_WRCLK_PORT PORTD  // Write clock port
#define HT1632_WRCLK_DDR  DDRD   // Write clock port
#endif

#ifdef HIGHPINS
// Port B defines for 8-13
byte ht1632_cs[4] = {8, 9, 10, 11};    // Chip Select (1, 2, 3, or 4)
volatile uint8_t* ht1632_cs_Port[4] = {&PORTB, &PORTB, &PORTB, &PORTB};    // Chip Select (1, 2, 3, or 4)
volatile uint8_t* ht1632_cs_DDR[4] = {&DDRB, &DDRB, &DDRB, &DDRB};    // Chip Select (1, 2, 3, or 4)
#define HT1632_DATA     12       // Data pin
#define HT1632_DATA_PORT  PORTB  // Data port
#define HT1632_DATA_DDR   DDRB   // Data port
#define HT1632_WRCLK    13       // Write clock pin
#define HT1632_WRCLK_PORT PORTB  // Write clock port
#define HT1632_WRCLK_DDR  DDRB   // Write clock port

#endif

// helper macros
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define chip_number(x,y) (x >> 5) + (y >> 3)*numYDevices
#define chip_nibble_address(x,y) ((x%32)<<1) + ((y%8)>>2);  
#define chip_byte_address(x,y) ((x%32)<<1);


// Display size and configuration, defaul is for a single 8x32 display
byte numDevices = 1;	// Total number of devices
byte numXDevices = 1;	// Number of horizontal devices
byte numYDevices = 1;	// Number of vertical devices
byte xMax = 32 * numXDevices-1;
byte yMax = 8 * numYDevices-1;

// Variables used to keep track of cursor position
int cursorX = 0;
int cursorY = 0;

/*
 * we keep a copy of the display controller contents so that we can
 * know which bits are on without having to (slowly) read the device.
 */
// 4 boards at 32 bytes per board + 1 byte means we don't need to check overwrite in putChar
byte shadowram[129];  // our copy of the display's RAM



// Custom character buffers - 8 characters available
// 6 cols * 8 rows - first byte of each char is the number of columns used
// Bits are aranged in columns with LSB at top
byte cgram [8][7] = { 
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };


// Default constructor
HT1632_LedMatrix::HT1632_LedMatrix( void ) {
}

void HT1632_LedMatrix::init( byte xDevices, byte yDevices ) {
  // Set up the display size based on number of devices both horizontal and vertical
  numXDevices = xDevices;
  xMax = 32 * numXDevices-1;
  numYDevices = yDevices;
  yMax = 8 * numYDevices-1;

  numDevices = numXDevices * numYDevices;

  HT1632_WRCLK_DDR |= _BV( HT1632_WRCLK );		// set pins to output
  HT1632_DATA_DDR |= _BV( HT1632_DATA );

  for (byte chipno=0; chipno<numDevices; chipno++){
	  *ht1632_cs_DDR[chipno] |= _BV( ht1632_cs[chipno] );	// output pin
	  chipfree(chipno); 	// unselect it 
	  sendcmd(chipno, HT1632_CMD_SYSDIS);  // Disable system
	  sendcmd(chipno, HT1632_CMD_COMS10);  // 08*32, PMOS drivers
	  sendcmd(chipno, HT1632_CMD_MSTMD); 	// Master Mode 
	  sendcmd(chipno, HT1632_CMD_SYSON); 	// System on 
	  sendcmd(chipno, HT1632_CMD_LEDON); 	// LEDs on 
	  sendcmd(chipno, HT1632_CMD_PWM | 0x0c); 	// PWM Duty 
	  for (byte i=0; i<96; i++)
		senddata(chipno, i, 0);  // clear the display
	  delay(100);
  }
  cursorX = 0;
  cursorY = 0;
}
 
/***********************************************************************
 * chipselect / chipfree
 * Select or de-select a particular ht1632 chip.
 * De-selecting a chip ends the commands being sent to a chip.
 * CD pins are active-low; writing 0 to the pin selects the chip.
 ***********************************************************************/
void HT1632_LedMatrix::chipselect(byte chipno)
{
#ifdef DIRECTIO
  output_low(*ht1632_cs_Port[chipno], ht1632_cs[chipno]);
#else
  digitalWrite( ht1632_cs[chipno], LOW );
#endif
}

void HT1632_LedMatrix::chipfree(byte chipno)
{
#ifdef DIRECTIO
  output_high(*ht1632_cs_Port[chipno], ht1632_cs[chipno]);
#else
  digitalWrite( ht1632_cs[chipno], HIGH );
#endif
}

/*
 * writebits
 * Write bits to h1632 on pins HT1632_DATA, HT1632_WRCLK
 * Chip is assumed to already be chip-selected
 * Bits are shifted out from MSB to LSB, with the first bit sent
 * being (bits & firstbit), shifted till firsbit is zero.
 */
void HT1632_LedMatrix::writebits (byte bits, byte firstbit)
{
  while (firstbit) {
#ifdef DIRECTIO
    output_low(HT1632_WRCLK_PORT, HT1632_WRCLK);
#else
    digitalWrite( HT1632_WRCLK, LOW );
#endif
    if (bits & firstbit) {
#ifdef DIRECTIO
      output_high(HT1632_DATA_PORT,HT1632_DATA);
#else
    digitalWrite( HT1632_DATA, HIGH);
#endif
    } else {
#ifdef DIRECTIO
      output_low(HT1632_DATA_PORT, HT1632_DATA);
#else
    digitalWrite( HT1632_DATA, LOW);
#endif
    }
#ifdef DIRECTIO
    output_high(HT1632_WRCLK_PORT, HT1632_WRCLK);
#else
    digitalWrite( HT1632_WRCLK, HIGH);
#endif
    firstbit >>= 1;
  }
}

/*
 * writedatabits
 * Write databits to h1632 on pins HT1632_DATA, HT1632_WRCLK
 * Chip is assumed to already be chip-selected
 * Bits are shifted out from LSB to MSB
 */
void HT1632_LedMatrix::writedatabits (byte bits, byte count)
{
  while (count) {
    output_low(HT1632_WRCLK_PORT, HT1632_WRCLK);
    if (bits & 1) {
      output_high(HT1632_DATA_PORT,HT1632_DATA);
    } else {
      output_low(HT1632_DATA_PORT, HT1632_DATA);
    }
    output_high(HT1632_WRCLK_PORT, HT1632_WRCLK);
    count--;
    bits >>= 1;
  }
}

/*
 * sendcmd
 * Send a command to the ht1632 chip.
 * A command consists of a 3-bit "CMD" ID, an 8bit command, and
 * one "don't care bit".
 *   Select 1 0 0 c7 c6 c5 c4 c3 c2 c1 c0 xx Free
 */
void HT1632_LedMatrix::sendcmd (byte chipno, byte command)
{
  chipselect(chipno);  // Select chip
  writebits(HT1632_ID_CMD, 0x04);	//1<<2);  // send 3 bits of id: COMMMAND
  writebits(command, 0x80);	//1<<7);  // send the actual command
  writebits(0, 1); 	/* one extra dont-care bit in commands. */
  chipfree(chipno); //done
}

/*
 * clear
 * clear the display, and the shadow memory, and the snapshot
 * memory.  This uses the "write multiple words" capability of
 * the chipset by writing all 96 words of memory without raising
 * the chipselect signal.
 */
void HT1632_LedMatrix::clear()
{
  char i;

  for (byte chipno=0; chipno<numDevices; chipno++){
    chipselect(chipno);  // Select chip
    writebits(HT1632_ID_WR, 0x04);	//1<<2);  // send ID: WRITE to RAM
    writebits(0, 0x40);		//1<<6); // Send address
    for (i = 0; i < 32; i++) // Clear entire display
      writedatabits(0, 8);	//1<<7); // send 8 bits of data
    chipfree(chipno); // done
    for (i=0; i < 64; i++)
      shadowram[i+64*chipno] = 0;
  }
  cursorX = 0;
  cursorY = 0;
}


// Brighness is from 0 to 15
void HT1632_LedMatrix::setBrightness( unsigned char brightness ) {
  for (byte chipno=0; chipno<numDevices; chipno++) {
	sendcmd(chipno, HT1632_CMD_PWM | (brightness & 0x0F ));
  }
}


/*
 * senddata
 * send a nibble (4 bits) of data to a particular memory location of the
 * ht1632.  The command has 3 bit ID, 7 bits of address, and 4 bits of data.
 *    Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 Free
 * Note that the address is sent MSB first, while the data is sent LSB first!
 * This means that somewhere a bit reversal will have to be done to get
 * zero-based addressing of words and dots within words.
 */
void HT1632_LedMatrix::senddata (byte chipno, byte address, byte data)
{
  chipselect(chipno);  // Select chip
  writebits(HT1632_ID_WR, 0x04);	//1<<2);  // send ID: WRITE to RAM
  writebits(address, 0x40);		//1<<6); // Send address
  writedatabits(data, 4);		//1<<3); // send 4 bits of data
  chipfree(chipno); // done
}

/*
 * sendcol
 * send a byte of data to a particular memory location of the
 * ht1632.  The command has 3 bit ID, 7 bits of address, and 8 bits of data.
 *    Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 D4 D5 D6 D7 D8 Free
 * Note that the address is sent MSB first, while the data is sent LSB first!
 * This means that somewhere a bit reversal will have to be done to get
 * zero-based addressing of words and dots within words.
 */
void HT1632_LedMatrix::sendcol (byte chipno, byte address, byte data)
{
  chipselect(chipno);  // Select chip
  writebits(HT1632_ID_WR, 0x04);	//1<<2);  // send ID: WRITE to RAM
  writebits(address, 0x40);		//1<<6); // Send address
  writedatabits(data, 8);		// send 8 bits of data
  chipfree(chipno); // done
}

// Write a string at the position specified
void HT1632_LedMatrix::putString(int x, int y, char *str) {
	cursorX = x;
	cursorY = y;
	while( *str ) {
		putChar( cursorX, y, *str++ );
		cursorX += 6;
	}
}

/*
 * Copy a character glyph from the smallFont data structure to
 * display memory, with its upper left at the given coordinate
 * This is unoptimized and simply uses plot() to draw each dot.
 */
void HT1632_LedMatrix::write( uint8_t c) {
	putChar( cursorX, cursorY, (char)c );
}

/*
 * Copy a character glyph from the myfont data structure to
 * display memory, with its upper left at the given coordinate
 * This is unoptimized and simply uses plot() to draw each dot.
 * returns number of columns that didn't fit
 */
byte HT1632_LedMatrix::putChar(int x, int y, char c) {
  // fonts defined for ascii 32 and beyond (index 0 in font array is ascii 32);
  // CGRAM characters are in range 0 to 15 with 8-15 being repeat of 0-7
  // note we force y to be modulo 8 - we do not support writing character to partial y values.

  byte charIndex;
  byte colData;
  byte numCols;
  byte chipno;
  byte addr;
  byte colsLeft = 0;		// cols that didn't fit

  if( c > 15 ) {
	// Regular characters
  	// replace undisplayable characters with blank;
  	if (c < 32 || c > 126) {
    		charIndex = 0;
  	} else {
    		charIndex = c - 32;
  	}

  	// move character definition, pixel by pixel, onto the display;
  	// fonts are defined as one byte per col;
	numCols=pgm_read_byte_near(&smallFont[charIndex][6]);	// get the number of columns this character uses
  	for (byte col=0; col<numCols; col++) {
    		colData = pgm_read_byte_near(&smallFont[charIndex][col]);
		chipno = chip_number(x,y);
		addr = chip_byte_address(x,y); // compute which memory byte this is in
		if (x <= xMax && y <= yMax) {
			shadowram[(addr>>1)+32*chipno] = colData;
			sendcol(chipno,addr,colData);
			x++;
		} else {
			colsLeft++;
		}
  	}
  } else {
	// CGRAM Characters
	charIndex = c & 0x07;		// Only low 3 bits count
	numCols=cgram[charIndex][0];	// get the number of columns this character uses
  	// fonts are defined as one byte per col;
    	for (byte col=1; col<=numCols; col++) {
    		colData = cgram[charIndex][col];
		chipno = chip_number(x,y);
		addr = chip_byte_address(x,y); // compute which memory byte this is in
		if (x <= xMax && y <= yMax) {
			shadowram[(addr>>1)+32*chipno] = colData;
			sendcol(chipno,addr,colData);
			x++;
		} else {
			colsLeft++;
		}
  	}
  }

  cursorX = x;
  cursorY = y;

  return colsLeft;
}

// Set position of cursor for writing
void HT1632_LedMatrix::gotoXY(int x, int y) {
	cursorX = x;
	cursorY = y;
}

void HT1632_LedMatrix::getXY(int* x, int* y)
{
	*x = cursorX;
	*y = cursorY;
}

void HT1632_LedMatrix::getXYMax(int* x, int* y)
{
	*x = xMax;
	*y = yMax;
}

// Shift cursor X position a number of positions either left or right.
void HT1632_LedMatrix::shiftCursorX(int xinc) {
	cursorX += xinc;
}


/*
 * plot a point on the display, with the upper left hand corner
 * being (0,0), and the lower right hand corner being (xMax-1, yMax-1).
 * Note that Y increases going "downward" in contrast with most
 * mathematical coordiate systems, but in common with many displays
 * basic bounds checking used.
 */
void HT1632_LedMatrix::plot (int x, int y, char val)
{
  if (x<0 || x>xMax || y<0 || y>yMax)
     return;

  byte chipno = chip_number(x,y);
  char addr = chip_byte_address(x,y); // compute which memory word this is in
  char shadowAddress = addr >>1;

  char bitval = 1<<(y&7);  // compute which bit will need set


  if (val) {  // Modify the shadow memory
    shadowram[shadowAddress +32*chipno] |= bitval;
  }
  else {
    shadowram[shadowAddress +32*chipno] &= ~bitval;
  }
  // Now copy the new memory value to the display
  sendcol(chipno, addr, shadowram[shadowAddress +32*chipno]);
}


void HT1632_LedMatrix::setCustomChar( int charNum, unsigned char cgchar[] ) {
	for(int i=1; i<7; i++ ) {
		cgram[charNum][i] = (byte)cgchar[i];
	}
	cgram[charNum][6] = 0;
	cgram[charNum][0] = 6;
}

void HT1632_LedMatrix::setCustomChar( int charNum, unsigned char cgchar[], byte numCols ) {
	numCols = max(numCols, 6 );
	for(int i=1; i<=numCols; i++ ) {
		cgram[charNum][i] = (byte)cgchar[i];
	}
	cgram[charNum][0] = numCols;
	cgram[charNum][numCols] = 0;
}

void HT1632_LedMatrix::scrollLeft(byte numberCols)
{
	for (int i=0; i<128-numberCols-1; i++)
	{
		shadowram[i]=shadowram[i+numberCols];
	}
	for (int i=128-numberCols; i<128; i++)
	{
		shadowram[i]=0;
	}
	cursorX -= numberCols;
	if (cursorX < 0 ) cursorX = 0;
}

void HT1632_LedMatrix::putShadowRam()
{
	for (int chipno=0; chipno<numDevices; chipno++)
		putShadowRam(chipno);
}

void HT1632_LedMatrix::putShadowRam(byte chipno)
{
	for (int i=0; i<64; i+=2)
	{
		sendcol(chipno,i,shadowram[(i>>1)+32*chipno]);
	}
}

#ifdef USE_GRAPHIC
/*
 * Name         : drawLine
 * Description  : Draws a line between two points on the display.
 * Argument(s)  : x1, y1 - Absolute pixel coordinates for line origin.
 *                x2, y2 - Absolute pixel coordinates for line end.
 *                c - either PIXEL_ON, PIXEL_OFF
 * Return value : none
 */
void HT1632_LedMatrix::drawLine(unsigned char x1, unsigned char y1,
		unsigned char x2, unsigned char y2, unsigned char c) {
    int dx, dy, stepx, stepy, fraction;

    /* Calculate differential form */
    /* dy   y2 - y1 */
    /* -- = ------- */
    /* dx   x2 - x1 */

    /* Take differences */
    dy = y2 - y1;
    dx = x2 - x1;

    /* dy is negative */
    if ( dy < 0 ) {
        dy    = -dy;
        stepy = -1;
    } else {
        stepy = 1;
    }

    /* dx is negative */
    if ( dx < 0 ) {
        dx    = -dx;
        stepx = -1;
    } else {
        stepx = 1;
    }

    dx <<= 1;
    dy <<= 1;

    /* Draw initial position */
    plot( x1, y1, c );

    /* Draw next positions until end */
    if ( dx > dy ) {
        /* Take fraction */
        fraction = dy - ( dx >> 1);
        while ( x1 != x2 ) {
            if ( fraction >= 0 ) {
                y1 += stepy;
                fraction -= dx;
            }
            x1 += stepx;
            fraction += dy;

            /* Draw calculated point */
            plot( x1, y1, c );
        }
    } else {
        /* Take fraction */
        fraction = dx - ( dy >> 1);
        while ( y1 != y2 ) {
            if ( fraction >= 0 ) {
                x1 += stepx;
                fraction -= dy;
            }
            y1 += stepy;
            fraction += dx;

            /* Draw calculated point */
            plot( x1, y1, c );
        }
    }
}


/*
 * Name         : drawRectangle
 * Description  : Draw a rectangle given to top left and bottom right points
 * Argument(s)  : x1, y1 - Absolute pixel coordinates for top left corner
 *                x2, y2 - Absolute pixel coordinates for bottom right corner
 *                c - either PIXEL_ON, PIXEL_OFF 
 * Return value : none
 */
void HT1632_LedMatrix::drawRectangle(unsigned char x1, unsigned char y1,
		unsigned char x2, unsigned char y2, unsigned char c){
	drawLine( x1, y1, x2, y1, c );
	drawLine( x1, y1, x1, y2, c );
	drawLine( x1, y2, x2, y2, c );
	drawLine( x2, y1, x2, y2, c );
}


/*
 * Name         : drawFilledRectangle
 * Description  : Draw a filled rectangle given to top left and bottom right points
 * 		  just simply draws horizontal lines where the rectangle would be
 * Argument(s)  : x1, y1 - Absolute pixel coordinates for top left corner
 *                x2, y2 - Absolute pixel coordinates for bottom right corner
 *                c - either PIXEL_ON, PIXEL_OFF
 * Return value : none
 */
void HT1632_LedMatrix::drawFilledRectangle(unsigned char x1, unsigned char y1,
		unsigned char x2, unsigned char y2, unsigned char c) {
	for(int i=y1; i <= y2; i++ ) {
		drawLine( x1, i, x2, i, c );
	}
}


/*
 * Name         : drawCircle
 * Description  : Draw a circle using Bresenham's algorithm. 
 * 		  Some small circles will look like squares!!
 * Argument(s)  : xc, yc - Centre of circle
 * 		  r - Radius
 * 		  c - either PIXEL_ON, PIXEL_OFF
 * Return value : None 
 */
void HT1632_LedMatrix::drawCircle(unsigned char xc, unsigned char yc,
		unsigned char r, unsigned char c) {
	int x=0;
	int y=r;
	int p=3-(2*r);

        plot( xc+x,yc-y, c);

	for(x=0;x<=y;x++) {
		if (p<0) {
			y=y;
			p=(p+(4*x)+6);
		} else {
			y=y-1;
			p=p+((4*(x-y)+10));
		}

		plot(xc+x,yc-y, c);
		plot(xc-x,yc-y, c);
		plot(xc+x,yc+y, c);
		plot(xc-x,yc+y, c);
		plot(xc+y,yc-x, c);
		plot(xc-y,yc-x, c);
		plot(xc+y,yc+x, c);
		plot(xc-y,yc+x, c);
	}
}
#endif

// The end!
