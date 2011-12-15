/*
*	HT1632_LedMatrix.h
*	defintions for Holtek ht1632 LED driver
*/

#ifndef _HT1632_LEDMATRIX_H
#define _HT1632_LEDMATRIX_H

#define USE_GRAPHIC

#if ARDUINO >= 100
  #include <Arduino.h> // Arduino 1.0
  #define WRITE_RESULT size_t
  #define WRITE_RETURN return 1;
#else
  #include <WProgram.h> // Arduino 0022
  #define WRITE_RESULT void
  #define WRITE_RETURN
#endif

#include <inttypes.h>
#include <avr/pgmspace.h>
#include "Print.h"

/*
 * commands written to the chip consist of a 3 bit "ID", followed by
 * either 9 bits of "Command code" or 7 bits of address + 4 bits of data.
 */
#define HT1632_ID_CMD 4		/* ID = 100 - Commands */
#define HT1632_ID_RD  6		/* ID = 110 - Read RAM */
#define HT1632_ID_WR  5		/* ID = 101 - Write RAM */

#define HT1632_CMD_SYSDIS 0x00	/* CMD= 0000-0000-x Turn off oscil */
#define HT1632_CMD_SYSON  0x01	/* CMD= 0000-0001-x Enable system oscil */
#define HT1632_CMD_LEDOFF 0x02	/* CMD= 0000-0010-x LED duty cycle gen off */
#define HT1632_CMD_LEDON  0x03	/* CMD= 0000-0011-x LEDs ON */
#define HT1632_CMD_BLOFF  0x08	/* CMD= 0000-1000-x Blink ON */
#define HT1632_CMD_BLON   0x09	/* CMD= 0000-1001-x Blink Off */
#define HT1632_CMD_SLVMD  0x10	/* CMD= 0001-00xx-x Slave Mode */
#define HT1632_CMD_MSTMD  0x14	/* CMD= 0001-01xx-x Master Mode */
#define HT1632_CMD_RCCLK  0x18	/* CMD= 0001-10xx-x Use on-chip clock */
#define HT1632_CMD_EXTCLK 0x1C	/* CMD= 0001-11xx-x Use external clock */
#define HT1632_CMD_COMS00 0x20	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_COMS01 0x24	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_COMS10 0x28	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_COMS11 0x2C	/* CMD= 0010-ABxx-x commons options */
#define HT1632_CMD_PWM    0xA0	/* CMD= 101x-PPPP-x PWM duty cycle */

#define PIXEL_OFF 0
#define PIXEL_ON  1

//class HT1632_LedMatrix 
class HT1632_LedMatrix  : public Print 
{
  private:
	void chipselect( byte );
	void chipfree( byte );
	void writebits( byte, byte );
	void writedatabits( byte, byte );
	void sendcmd( byte, byte );
	void senddata( byte, byte, byte );
	void sendcol( byte, byte, byte );

  public:
  	HT1632_LedMatrix( );
  	
  	// Init/Clear/position functions
	void init( byte, byte );
	void clear(void);
	void setBrightness( unsigned char );
	byte putChar( int, int, char );
	WRITE_RESULT write( uint8_t );
	void putString( int, int, char* );
	void plot( int, int, char );
	void gotoXY(int , int);
	void getXY(int* , int*);
	void getXYMax(int*, int*);
	void shiftCursorX(int );
	void setCustomChar( int, unsigned char[]);
	void setCustomChar( int, unsigned char[], byte );
	void scrollLeft(byte);
	void putShadowRam();
	void putShadowRam(byte);
	// Graphic functions
#ifdef  USE_GRAPHIC
	void drawLine(unsigned char x1, unsigned char y1,
			unsigned char x2, unsigned char y2, unsigned char c);
	void drawRectangle(unsigned char x1, unsigned char y1,
			unsigned char x2, unsigned char y2, unsigned char c);
	void drawFilledRectangle(unsigned char x1, unsigned char y1,
			unsigned char x2, unsigned char y2, unsigned char c);
	void drawCircle(unsigned char xc, unsigned char yc,
			unsigned char r, unsigned char c);
#endif

};

#endif //_HT1632_LEDMATRIX_H

