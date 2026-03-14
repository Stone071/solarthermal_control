// serial 8 bit LCD interface for ECE-3411
// By John Chandy, 2020
// API based on 8-bit/4-bit LCD interface from Scienceprog.com
// Modified by Zachary Stone

#include <stdint.h>

void LcdInitialize(void);          //Initializes LCD
void LcdDataWrite(uint8_t);        //Send one byte over LCD_SERIAL_MODE
void LcdClear(void);               //Clears LCD
void LcdHome(void);                //LCD cursor home
void LcdString(char*, uint8_t);    //Outputs string to LCD
void LcdGoToXY(uint8_t, uint8_t);  //Cursor to X Y position
void LcdFlashString(const uint8_t*, uint8_t, uint8_t);//copies flash string to LCD at x,y
void LcdShiftRight(uint8_t);       //shift by n characters Right
void LcdShiftLeft(uint8_t);        //shift by n characters Left
void LcdCursorOnUnderline(void);   //Underline cursor ON
void LcdCursorOnBlink(void);       //Blinking cursor ON
void LcdCursorOff(void);           //Both underline and blinking cursor OFF
void LcdBlank(void);               //LCD blank but not cleared
void LcdVisible(void);             //LCD visible
void LcdCursorLeft(uint8_t);       //shift cursor left by n
void LcdCursorRight(uint8_t);      //shift cursor right by n