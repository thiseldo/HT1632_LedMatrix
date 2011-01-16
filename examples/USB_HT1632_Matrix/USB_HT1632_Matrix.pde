/*
 * USB HT1632 Driver using a basic version of Matrix Orbital protocol.
 * Allows a number of HT1632 based LED matrix displays to be 
 * A. D. Lindsay 02/10/2008
 */
#define USE_NSS
#define SERIAL_SPEED 19200

#include "HT1632_LedMatrix.h"
#ifdef USE_NSS
#include <NewSoftSerial.h>

// Use NewSoftSerial for 
NewSoftSerial ledSerial(0, 1);
#endif

// create object to control the LED Matrix
HT1632_LedMatrix led = HT1632_LedMatrix();

// Barwidth for vertical bars
int barWidth = 3;


void setup() { 
#ifdef USE_NSS
  ledSerial.begin( SERIAL_SPEED );
#else
  Serial.begin( SERIAL_SPEED );
#endif

  // Initialise LED, 2 displays wide, 1 high
  led.init( 2, 1 );
  // set to maximum brightness
  led.setBrightness( 0x0f );
  led.print("USBDispV.4");
  delay(1500);
  led.clear();
}


byte serial_getch(){
  int incoming;  

#ifdef USE_NSS
  while( ledSerial.available() == 0 );
  // read the incoming byte:
  incoming = ledSerial.read();
#else
  while (Serial.available()==0);
  // read the incoming byte:
  incoming = Serial.read();
#endif

  return (byte) (incoming &0xff);
}


void loop(){
  int xPos, yPos;
  byte rxbyte;
  byte temp;
  int dir, barlen;
  unsigned char cgchar[8];

  rxbyte = serial_getch();

  //Matrix Orbital uses 254 prefix for commands
  if (rxbyte == 254) {
    switch (serial_getch())	{
    case 66: //backlight on (at previously set brightness)
      // not implemented				
      temp = serial_getch();  //gobble byte
      break;
    case 70: //backlight off
      // not implemented				
      break;
    case 71:  //set cursor position
      temp = (serial_getch() - 1);  //get column byte
      xPos = (int)temp * 6;
      temp = (serial_getch() - 1);    // get row byte
      yPos = (int)temp * 8;
      led.gotoXY(xPos, yPos);
      break;
    case 72:  //cursor home (reset display position) Is row 1, column 1 on MatrixOrbital Displays
      led.gotoXY( 0, 0);
      break;
    case 76:  //move cursor left
      led.shiftCursorX( -6 );
      break;
    case 77:  //move cursor right
      led.shiftCursorX( 6 );
      break;
    case 78:  //define custom char
      temp = (int)(serial_getch());  // Get character ref
      // Read 8 bytes
      for (int i = 0; i< 8; i++ ) {
        cgchar[i] = (unsigned char)(serial_getch());
      }
      led.setCustomChar( temp, cgchar );
      break;
    case 88:  //clear display, cursor home
      led.clear();
      break;
    case 192:  //Load custom char - just gobble for now
      temp = (int)(serial_getch());  // Get bank
      break;
    case 193:  //Save custom char - just gobble for now
      temp = (int)(serial_getch());  // bank
      temp = (int)(serial_getch());  // character ref
      // Read 8 bytes
      for (int i = 0; i< 8; i++ ) {
        cgchar[i] = (unsigned char)(serial_getch());
      }
      break;
    case 194:  //Save startup char - just gobble for now
      temp = (int)(serial_getch());  // character ref
      // Read 8 bytes
      for (int i = 0; i< 8; i++ ) {
        cgchar[i] = (unsigned char)(serial_getch());
      }
      //      led.setCustomChar( temp, cgchar );
      break;
    case 109:  //Initialise medium char - just gobble for now
      temp = (int)(serial_getch());  // bank
      //      led.setCustomChar( temp, cgchar );
      break;
    case 111:  //Place medium char - just gobble for now
      temp = (int)(serial_getch());  // Row
      temp = (int)(serial_getch());  // Column
      temp = (int)(serial_getch());  // number 0-9
      break;
    case 104:  // Initialise horizontal bar
      break;
    case 124:  //Place horizontal bar
      temp = (serial_getch() - 1);  //get column byte
      xPos = (int)temp * 6;
      temp = (serial_getch() - 1);    // get row byte
      yPos = (int)temp * 8;

      dir = (int)(serial_getch());  // Direction 0 or 1. 0 goes right, 1 goes left
      barlen = (int)(serial_getch());  // Length
      for( int i = 0; i < barlen; i++ ) {
        for( int n = 0; n < 8; n++ ) {
          if( dir == 0 ) {
            led.plot( xPos + i, yPos + n, 1 );
          } 
          else {
            led.plot( xPos - i, yPos + n, 1 );
          }
        }
      }
      break;
    case 115:  // Initialise narrow vertical bar
      barWidth = 3;
      break;
    case 118:  // Initialise wide vertical bar
      barWidth = 6;
      break;
    case 61:  //Place vertical bar
      temp = (int)(serial_getch() - 1);  // Column
      xPos = (int)temp * 6;
      barlen = (int)(serial_getch());  // Length
      for( int i = 0; i < barlen; i++ ) {
        for( int n = 0; n < barWidth; n++ ) {
          led.plot( xPos + n, 7-i, 1 );
        }
      }
      break;      
    case 37:  //GPO Mode
    case 86:  //GPO OFF
    case 87:  //GPO ON
      temp = (int)(serial_getch());
      break;
    case 195:  //GPO Startup state
      temp = (int)(serial_getch());
      temp = (int)(serial_getch());
      break;

    case 74:  //show underline cursor
    case 75:  //underline cursor off
    case 83:  //show blinking block cursor
    case 84:  //block cursor off
      break;

    case 153: //set backlight brightness
    case 152: //set and remember (doesn't save value, though)
      temp = serial_getch();
      led.setBrightness( temp / 16 );
      break;

    case 80:  // set contrast
    case 145: // set and save contrast
      //not implemented
      temp = serial_getch();  //gobble byte
      break;
    case 54: //read version number
      //      Serial.print( 'A' );
      break;
    case 55: //read module type
      //      Serial.print( '\0' );
      break;

      //these commands ignored (no parameters)
    case 35: //read serial number
    case 59: //exit flow-control mode
    case 64: //Change startup text
    case 65: //auto transmit keypresses
    case 96: //auto-repeat mode off (keypad)
    case 67: //auto line-wrap on
    case 68: //auto line-wrap off
    case 81: //auto scroll on
    case 82: //auto scroll off
      break;
    default:
      //all other commands ignored and parameter byte discarded
      temp = serial_getch();  //dump the command code
      break;
    }
    return;
  } //END OF COMMAND HANDLER

  //change accented char to plain, detect and change descenders
  switch (rxbyte) {
  case 0x08:    // Backspace
    break;
  case 0x0c:    // Clear Screen
    led.clear();
    break;
  case 0x0D:    // Carriage Return
    break;
  case 0x0A:    // Line feed
    break;
  case 0xE4: //ASCII "a" umlaut
    rxbyte = 0xE1;
    break;
  case 0xF1: //ASCII "n" tilde
    rxbyte = 0xEE;
    break;
  case 0xF6: //ASCII "o" umlaut
    rxbyte = 0xEF;
    break;
  case 0xFC: //ASCII "u" umlaut
    rxbyte = 0xF5;
    break;

    //accented -> plain equivalent
    //and misc symbol translation
  case 0xA3: //sterling (pounds)
    rxbyte = 0xED;
    break;
    /*
  	case 0xB0: //degrees symbol
     			rxbyte = 0xDF;
     			break;
     */
  case 0xB5: //mu
    rxbyte = 0xE4;
    break;
  case 0xC0: //"A" variants
  case 0xC1:
  case 0xC2:
  case 0xC3:
  case 0xC4:
  case 0xC5:
    rxbyte = 0x41;
    break;
  case 0xC8: //"E" variants
  case 0xC9:
  case 0xCA:
  case 0xCB:
    rxbyte = 0x45;
    break;
  case 0xCC: //"I" variants
  case 0xCD:
  case 0xCE:
  case 0xCF:
    rxbyte = 0x49;
    break;
  case 0xD1: //"N" tilde -> plain "N"
    rxbyte = 0x43;
    break;
  case 0xD2: //"O" variants
  case 0xD3:
  case 0xD4:
  case 0xD5:
  case 0xD6:
  case 0xD8:
    rxbyte = 0x4F;
    break;
  case 0xD9: //"U" variants
  case 0xDA:
  case 0xDB:
  case 0xDC:
    rxbyte = 0x55;
    break;
  case 0xDD: //"Y" acute -> "Y"
    rxbyte = 0x59;
    break;
    /*
  	case 0xDF: //beta  //mucks up LCDSmartie's degree symbol??
     		rxbyte = 0xE2;
     		break;
     */
  case 0xE0: //"a" variants except umlaut
  case 0xE1:
  case 0xE2:
  case 0xE3:
  case 0xE5:
    rxbyte = 0x61;
    break;
  case 0xE7: //"c" cedilla -> "c"
    rxbyte = 0x63;
    break;
  case 0xE8: //"e" variants
  case 0xE9:
  case 0xEA:
  case 0xEB:
    rxbyte = 0x65;
    break;
  case 0xEC: //"i" variants
  case 0xED:
  case 0xEE:
  case 0xEF:
    rxbyte = 0x69;
    break;
  case 0xF2: //"o" variants except umlaut
  case 0xF3:
  case 0xF4:
  case 0xF5:
  case 0xF8:
    rxbyte = 0x6F;
    break;
  case 0xF7: //division symbol
    rxbyte = 0xFD;
    break;
  case 0xF9: //"u" variants except umlaut
  case 0xFA:
  case 0xFB:
    rxbyte = 0x75;
    break;
  default:
    break;
  }

  led.print(rxbyte);  //otherwise a plain char so we print it to lcd
  return;
}



