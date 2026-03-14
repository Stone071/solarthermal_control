// ###############################################
// # lcd_lib.c
// #
// # Functions for controlling a New Haven display
// # 16x2 LCD.
// #
// # Zachary Stone, October 2021
// ###############################################

// INCLUDES
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> // For accessing the flash!
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <util/delay.h>
#include <math.h>

// DEFINES
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define LCD_CMD_DISP_ON    0x41
#define LCD_CMD_DISP_OFF   0x42
#define LCD_CMD_GOTO       0x45
#define LCD_CMD_HOME       0x46
#define LCD_CMD_CURS_UL    0x47
#define LCD_CMD_CURS_ULOFF 0x48
#define LCD_CMD_CURS_L     0x49
#define LCD_CMD_CURS_R     0x4A
#define LCD_CMD_CURS_BL    0x4B
#define LCD_CMD_CURS_BLOFF 0x4C
#define LCD_CMD_CLEAR      0x51
#define LCD_CMD_SHL        0x55
#define LCD_CMD_SHR        0x56
#define LCD_CMD_PREFIX     0xFE

// UartOneInit initializes Uart 1 for LCD communication
void UartOneInit(unsigned int ubrr)
{
    // Set the baud rate
    UBRR1H = (unsigned char) (ubrr>>8);
    UBRR1L = (unsigned char) ubrr;
    // Enable TXEN1
    UCSR1B = (1<<TXEN1);
    // Set the frame format for 8N1
    // I think this is default already.

    // Setting TXEN1 completely overrides any other general I/O settings for that pin.
}

// LcdInitialize initializes the UART comms then waits
void LcdInitialize(void)
{
    // Begin uart communication
    UartOneInit(MYUBRR);
    _delay_ms(500);
}

// LcdDataWrite places a byte into UDR1 for sending and waits until it is sent
void LcdDataWrite(uint8_t data)
{
    UDR1 = data;
    loop_until_bit_is_set(UCSR1A, UDRE1);
    // UDRE1 is flag which raises when usart 1 data register
    // is empty, so data has been sent.
}

// LcdClear clears the LCD display
void LcdClear(void)
{
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_CLEAR);
    _delay_us(1500);
}

// LcdPosHome moves the LCD control to home position
void LcdPosHome(void)
{
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_HOME);
    _delay_us(1500);
}

// LcdString sends len number of bytes from *str over LcdDataWrite
void LcdString(char* str, uint8_t len)
{
    unsigned int i = 0;
    while (i<len) 
    {
        LcdDataWrite(str[i]);
        i++;
    }
}

// LcdGoToXY sends the cursor to the position x,y where 0,0 is top left corner
void LcdGoToXY(uint8_t x, uint8_t y)
{
    uint8_t pos = 0x00;
    uint8_t offset = 0x00;
    if (y==1)  offset = 0x40;

    if (x==0)       pos = 0x00 + offset;
    else if (x==1)  pos = 0x01 + offset;
    else if (x==2)  pos = 0x02 + offset;
    else if (x==3)  pos = 0x03 + offset;
    else if (x==4)  pos = 0x04 + offset;
    else if (x==5)  pos = 0x05 + offset;
    else if (x==6)  pos = 0x06 + offset;
    else if (x==7)  pos = 0x07 + offset;
    else if (x==8)  pos = 0x08 + offset;
    else if (x==9)  pos = 0x09 + offset;
    else if (x==10) pos = 0x0A + offset;
    else if (x==11) pos = 0x0B + offset;
    else if (x==12) pos = 0x0C + offset;
    else if (x==13) pos = 0x0D + offset;
    else if (x==14) pos = 0x0E + offset;
    else if (x==15) pos = 0x0F + offset;
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_GOTO);
    LcdDataWrite(pos);
    _delay_us(100);
}

// LcdFlashString copies a string from flash memory and displays it on screen at x,y
void LcdFlashString(const uint8_t* FlashLoc, uint8_t x, uint8_t y)
{
    uint8_t i = 0;
    LcdGoToXY(x,y);
    for (i=0; (uint8_t)pgm_read_byte(&FlashLoc[i]); i++) 
    {
        LcdDataWrite((uint8_t)pgm_read_byte(&FlashLoc[i]));
    }
}

// LcdShiftRight moves lcd position right n times
void LcdShiftRight(uint8_t n)
{
    unsigned int i = 0;
    while (i<n) 
    {
        LcdDataWrite(LCD_CMD_PREFIX);
        LcdDataWrite(LCD_CMD_SHR);
        _delay_us (100);
        i++;
    }
}

// LcdShiftLeft moves lcd position left n times
void LcdShiftLeft(uint8_t n)
{
    unsigned int i = 0;
    while (i<n) 
    {
        LcdDataWrite(LCD_CMD_PREFIX);
        LcdDataWrite(LCD_CMD_SHL);
        _delay_us (100);
        i++;
    }
}

// LcdCursorOnUnderline moves places an underline where the LCD cursor is
void LcdCursorOnUnderline(void)
{
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_CURS_UL);
    _delay_us(1500);
}

// LcdCursorOnBlink makes the cursor blink
void LcdCursorOnBlink(void)
{
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_CURS_BL);
    _delay_us(100);
}

// LcdCursorOff turns off the visible cursor
void LcdCursorOff(void)
{
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_CURS_ULOFF); // cursor underline off
    _delay_us(1500);
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_CURS_BLOFF); // blinking cursor off
    _delay_us(100);
}

// LcdBlank turns off the LCD screen
void LcdBlank(void)
{
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_DISP_OFF); // display off
    _delay_us(100);
}

// LcdVisible turns on the LCD screen
void LcdVisible(void)
{
    LcdDataWrite(LCD_CMD_PREFIX);
    LcdDataWrite(LCD_CMD_DISP_ON); // display on
    _delay_us(100);
}

// LcdCursorLeft moves the cursor left n times
void LcdCursorLeft(uint8_t n)
{
    unsigned int i = 0;
    while (i<n) 
    {
        LcdDataWrite(LCD_CMD_PREFIX);
        LcdDataWrite(LCD_CMD_CURS_L);
        _delay_us (100);
        i++;
    }
}

// LcdCursorRight moves the cursor right n times
void LcdCursorRight(uint8_t n)
{
    unsigned int i = 0;
    while (i<n) 
    {
        LcdDataWrite(LCD_CMD_PREFIX);
        LcdDataWrite(LCD_CMD_CURS_R);
        _delay_us (100);
        i++;
    }
}