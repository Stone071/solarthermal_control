// ###############################################
// # solarthermal_control.c
// #
// # POC for limiting solar input to solarthermal
// # array at Spring Valley Student Farm.
// #
// # Zachary Stone, November 2021
// ###############################################

// INCLUDES
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd_lib.h"
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdlib.h> // contains dtostrf();
#include <avr/pgmspace.h>
#include <math.h>
#include <stdint.h>

// DEFINES
// I2C communication
#define JOY_ADDR  0x20 // i2c addr
#define X_REG     0x03 // i2c registers
#define Y_REG     0x05 // i2c registers
#define CLICK_REG 0x07 // i2c registers
#define WRITE_BIT 0    // i2c control
#define READ_BIT  1    // i2c control
// Motor Control
#define STEP_CW   1 // motor directions
#define STEP_CCW -1 // motor directions
#define IN1 PORTD0  // motor step pins
#define IN2 PORTD2  // motor step pins
#define IN3 PORTD3  // motor step pins
#define IN4 PORTD4  // motor step pins
// Photosensor Related
#define ERR_TOL 0.1 // photosensor voltage tolerance
#define ADC_DELAY 200 // ADC read delay
// User Interface
#define NAVIGATING_MENUS    0 // Values for uiState
#define ADJ_SHADE_POS       1 // Values for uiState
#define ADJ_BRIGHT_SETPOINT 2 // Values for uiState
#define POSITIVE_LEAN 192 // joystick sensitivity
#define NEGATIVE_LEAN  64 // joystick sensitivity
#define JOY_CLICKED     0 // joystick toggle
#define JOY_DEBOUNCE  500 // Joystick read timer
// LCD
#define MAX_HORIZ_POS  15 // LCD position limit
#define MAX_VERT_POS    1 // LCD position limit

// DATA TYPES
typedef enum {AUTO, MANUAL} controlMode; // controlMode can have value AUTO or MANUAL
struct CursorPos {
    uint8_t x;
    uint8_t y;
};

// GLOBALS
controlMode sysMode = MANUAL; // The overall control mode
static uint8_t uiState = NAVIGATING_MENUS; // The state of the ui
const struct CursorPos cursorControlModePosition = {7,1};
const struct CursorPos cursorAdjustPosition = {9,0};
uint8_t lcdPosX = 0;
uint8_t lcdPosY = 0;
float photocellVoltage;
uint8_t voltageSetpoint = 0;
volatile uint16_t adcReadTimer;
volatile bool adcReadFlag;
volatile bool lcdUpdateFlag;
uint16_t debounceTime;
// All text used for display -- PROGMEM attribute places in flash
const uint8_t voltageMsg[] PROGMEM = "V: \0";
const uint8_t controlMsg[] PROGMEM = "Cntrl: \0";
const uint8_t setpointMsg[] PROGMEM = "Set: \0";
const uint8_t adjMsg[] PROGMEM = "Adjust\0";
const uint8_t motorAdjMsgOne[] PROGMEM = "Adj shade with\0";
const uint8_t motorAdjMsgTwo[] PROGMEM = "joystick.\0";

// StopWdt disables any already-configured watchdog
void StopWdt(void)
{
    wdt_reset();                    // Reset watchdog timer
    MCUSR &= ~(1<<WDRF);            // Shut off watchdog reset flag
    WDTCSR |= (1<<WDCE) | (1<<WDE); // Watchdog change enable and watchdog enable
    WDTCSR = 0x00;                  // Disable watchdog
}

// InitTimer0 sets TC0 for 500us resolution.
void InitTimer0(void)
{
    TCCR0A |= (1<<WGM01); // Set TC0 to CTC mode
    OCR0A = 124;          // Count up to 124
    TIMSK0 = (1<<OCIE0A); // Enable timer 0 compare A ISR
    TCCR0B = 3;           // Set scaling to divide by 64 counts and start timer.
}

// TC0 interrupt. Executes every 500us.
ISR(TIMER0_COMPA_vect)
{
    if (adcReadTimer > 0)
    {
        adcReadTimer--;
    }
    else
    {
        adcReadFlag = true;   // I ended up grouping these two on the same timer
        lcdUpdateFlag = true;
        adcReadTimer = ADC_DELAY;
    }
    if (debounceTime > 0)
    {
        debounceTime--;       // This is to control how often the joystick position is checked.
    }                         // Otherwise the joystick provides input at full cycle speed.
}

// MotorControlInit sets the motor control pins for output
void MotorControlInit(void)
{
    // Set data direction register for output on motor control pins
    DDRD = (1<<PIND0 | 1<<PIND2 | 1<<PIND3 | 1<<PIND4);
}

// StepMotor takes a direction input and steps once in that direction
void StepMotor(int direction)
{
    // Persistent step index
    static int8_t stepIndex = 0;
    // Successive values for PORTD to step the motor
    static const uint8_t stepTable[4] = {(1<<IN1),(1<<IN2),(1<<IN3),(1<<IN4)};

    stepIndex += direction;

    // Handle looping of stepTable
    if (stepIndex > 3) stepIndex = 0;
    else if (stepIndex < 0) stepIndex = 3;
    // Zero relevant bits before setting, then set the output
    PORTD &= ~((1<<IN1)|(1<<IN2)|(1<<IN3)|(1<<IN4));
    PORTD |= stepTable[stepIndex];
}

// AutoMotorTask steps the motor to minimize the photocell feedback signal
void AutoMotorTask(void)
{
    // Photocells are being used in a voltage divider setup such that more light
    // creates a higher measured voltage.
    float err = voltageSetpoint - photocellVoltage; // Compute difference.
    if (fabs(err) > ERR_TOL)
    {
        if (err > 0) StepMotor(STEP_CCW); // not enough light, open the shutter
        else if (err < 0) StepMotor(STEP_CW); // too much light, close the shutter
    }
}

// TwiMasterInit configures registers for I2C operation
void TwiMasterInit(void)
{
    TWBR1 = 92; 	    // 16MHz clock; prescaler=1; SCLK=80KHz
    TWDR1 = 0xFF;       // Default data content, SDA released
    TWCR1 = (1<<TWEN);  // Enable TWI & Acknowledgments.
}

// TwiRead tells the target at Address which Data we want, then reads and returns the data
int8_t TwiRead(uint8_t Address, uint8_t Data)
{
    // First we must write to target to tell it which data we want
    TWCR1 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Send the START condition
    while(!(TWCR1 & (1<<TWINT)));                // Wait for TWINT to set

    TWDR1 = (Address<<1) | (WRITE_BIT); // Load in the address and write bit
    TWCR1 = (1<<TWINT) | (1<<TWEN); // clear the interrupt to begin transmission
    while(!(TWCR1 & (1<<TWINT)));   // wait for TWINT to set

    TWDR1 = Data;					// Load data in
    TWCR1 = (1<<TWINT) | (1<<TWEN); // clear interrupt
    while(!(TWCR1 & (1<<TWINT)));   // wait for twint to set

    // Instead of stopping here, we now begin a read

    // Now read the data
    TWCR1 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // send another start condition
    while(!(TWCR1 & (1<<TWINT)));				 // wait for TWINT

    TWDR1 = (Address<<1) | (READ_BIT);           // load in the address and read bit
    TWCR1 = (1<<TWINT) | (1<<TWEN);              // clear twint
    while(!(TWCR1 & (1<<TWINT)));                // wait for twint

    TWCR1 = (1<<TWINT) | (1<<TWEN);				 // clear twint
    while (!(TWCR1 & (1<<TWINT)));				 // wait for twint
    uint8_t readOut = TWDR1;					 // read the received value

    TWCR1 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // transmit STOP
    return readOut;								 // return the readout value
}

// UpdateCursorPos updates globals lcdPosX and lcdPosY based on input lean of joystick
void UpdateCursorPos(uint8_t horizIn, uint8_t vertIn)
{
    // Update horizontal position based on joystick input
    if (horizIn > POSITIVE_LEAN && lcdPosX < MAX_HORIZ_POS) lcdPosX++; // collide with edge of screen, not loop
    else if (horizIn < NEGATIVE_LEAN && lcdPosX > 0) lcdPosX--;

    // Update vertical position based on joystick input
    if (vertIn > POSITIVE_LEAN && lcdPosY < MAX_VERT_POS) lcdPosY++;
    else if (vertIn < NEGATIVE_LEAN && lcdPosY > 0) lcdPosY--;
}

// Check if joystick cursor is hovering over the input desiredPosition
bool CheckCursorPos(struct CursorPos desiredPosition)
{
    return (bool)(lcdPosX == desiredPosition.x && lcdPosY == desiredPosition.y);
}

// JoystickTask operates a state machine to handle user input via the joystick
void JoystickTask(void)
{
    // Reading in X,Y, and click of joystick.
    uint8_t joyHoriz = TwiRead(JOY_ADDR, X_REG);
    uint8_t joyVert = TwiRead(JOY_ADDR, Y_REG);
    uint8_t click = TwiRead(JOY_ADDR, CLICK_REG);
    // Save the position of the cursor from the previous call
    uint8_t tempX = lcdPosX;
    uint8_t tempY = lcdPosY;

    switch(uiState)
    {
        case NAVIGATING_MENUS:
        {
            // Joystick controls cursor position, SM monitors for clicks on known menu positions
            if (debounceTime == 0)
            {
                UpdateCursorPos(joyHoriz, joyVert); // Updates lcdPosX and lcdPosY
                if (tempX != lcdPosX || tempY  != lcdPosY) 
                {
                    debounceTime = JOY_DEBOUNCE; // debounce if moved
                    LcdGoToXY(lcdPosX,lcdPosY);  // Send the LCD cursor to the new position.
                }
                else if (click == JOY_CLICKED)
                {
                    debounceTime = JOY_DEBOUNCE;
                    if (CheckCursorPos(cursorControlModePosition))
                    {
                        sysMode = (sysMode == AUTO) ? MANUAL : AUTO; // Change to other mode
                        LcdClear();
                    }
                    else if (CheckCursorPos(cursorAdjustPosition))
                    {
                        uiState = (sysMode == AUTO) ? ADJ_BRIGHT_SETPOINT : ADJ_SHADE_POS;
                        LcdClear();
                    }
                }
            }
            break;
        }
        case ADJ_SHADE_POS:
        {
            // Exiting manual adjustment
            if (click == JOY_CLICKED && debounceTime == 0)
            {
                // return to navigating menus
                debounceTime = JOY_DEBOUNCE;
                LcdClear(); // clear LCD for menu change
                uiState = NAVIGATING_MENUS;
            }
            // Joystick controls shade position directly
            else
            {
                // if joystick up, curtain up
                if (joyVert > POSITIVE_LEAN) StepMotor(STEP_CCW);
                // if joystick down, curtain down
                else if (joyVert < NEGATIVE_LEAN) StepMotor(STEP_CW);
            }
            break;
        }
        case ADJ_BRIGHT_SETPOINT:
        {
            // all actions in here should be debounced
            if (debounceTime == 0)
            {
                // User exiting menu
                if (click == JOY_CLICKED)
                {
                    debounceTime = JOY_DEBOUNCE;
                    LcdClear();
                    uiState = NAVIGATING_MENUS;
                }
                // Joystick controlling voltageSetpoint for automatic shade operation
                else if (joyHoriz > POSITIVE_LEAN)
                {
                    // Allow user to thumb through numbers 1-5 as setpoints.
                    if (voltageSetpoint < 5) voltageSetpoint++; // Don't go above 5.
                    debounceTime = JOY_DEBOUNCE;
                }
                else if (joyHoriz < NEGATIVE_LEAN)
                {
                    if (voltageSetpoint > 0) voltageSetpoint--; // Don't go below 0.
                    debounceTime = JOY_DEBOUNCE;
                }
            }
            break;
        }
    }
}

// InitAdc initializes the on board ADC
void InitAdc(void)
{
    // ON PC4
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    // ADEN enables ADC
    // ADPS is prescaler -- 111 is div by 128.
    ADCSRA |= (1<<ADSC); // begins the conversion
}

// ReadAdcChannel reads the ADC given by input channel
uint16_t ReadAdcChannel(uint8_t channel)
{
    ADMUX = channel;            // specify which ADC to look at
    ADCSRA |= (1<<ADSC);        // begin a read
    while (ADCSRA & (1<<ADSC)); // ADSC clears when read is complete. Should be 13 cycles(812ns)
    return ADC;
}

// ReadAdc reads the four photocells and returns the average in volts
float ReadAdc(void)
{
    uint16_t sum = 0; // ADC has 10 bit precision
    uint8_t i;
    float avg;
    // Sum the reading across 4 photocells
    for (i = 0; i < 4; i++)
    {
        sum += ReadAdcChannel(i);
    }
    avg = sum/4.00;
    // Return the average in terms of voltage across cells
    return ((float)avg/1023.00)*5.00;
}

// UserInterfaceTask operates a state machine to display all needed prompts on the LCD
void UserInterfaceTask(void)
{
    char charBuf[4];

    // Display is dependent on uiState and sysMode
    switch (uiState)
    {
        case NAVIGATING_MENUS:
        {
            // Display "V: " at specified position
            LcdFlashString(voltageMsg,0,0);
            // Convert photocellVoltage to string and copy to display
            dtostrf(photocellVoltage,4,3,charBuf);
            LcdString(charBuf,4);
            if (sysMode == MANUAL)
            {
                //################
                //V:x.xxx   Adjust
                //Cntrl: Manual
                //################
                LcdFlashString(adjMsg,9,0);
                LcdFlashString(controlMsg,0,1);
                LcdString("Manual",6);
            }
            else if (sysMode == AUTO)
            {
                //################
                //V:x.xxx   Set:y
                //Cntrl: Auto
                //################
                LcdFlashString(setpointMsg,9,0);
                dtostrf(voltageSetpoint,1,0,charBuf);
                LcdString(charBuf,1);
                LcdFlashString(controlMsg,0,1);
                LcdString("Auto",4);
            }
            break;
        }
        case ADJ_SHADE_POS:
        {
            //################
            //Adj shade with
            //joystick.
            //################
            LcdFlashString(motorAdjMsgOne,0,0);
            LcdFlashString(motorAdjMsgTwo,0,1);
            break;
        }
        case ADJ_BRIGHT_SETPOINT:
        {
            //################
            //Setpoint: x y z
            //
            //################
            LcdFlashString(setpointMsg,0,0);
            // Convert num to str and copy to display
            dtostrf(voltageSetpoint,1,0,charBuf);
            LcdGoToXY(7,0);
            LcdString(charBuf,1);
            // Below we are creating a rotating display of numbers 1-5 the user can thumb through.
            // Example "Setpoint: 2 3 4" where prevVal = 2, voltageSetpoint = 3, nextVal = 4.
            if (voltageSetpoint > 0)
            {
                // only create the previous value if voltageSetpoint is 1 or greater.
                uint8_t prevVal = voltageSetpoint - 1;
                dtostrf(prevVal,1,0,charBuf);
                LcdGoToXY(5,0);
                LcdString(charBuf,1);
            }
            else
            {
                LcdGoToXY(5,0);
                LcdString(" ",1); // Clear the space if voltageSetpoint == 0.
            }
            if (voltageSetpoint < 5)
            {
                // only create the next value if voltageSetpoint is 4 or lesser.
                uint8_t nextVal = voltageSetpoint + 1;
                dtostrf(nextVal,1,0,charBuf);
                LcdGoToXY(9,0);
                LcdString(charBuf,1);
            }
            else
            {
                LcdGoToXY(9,0);
                LcdString(" ",1); // Clear the space if voltageSetpoint == 5.
            }
            break;
        }
    }
}

// Entry point of program
int main(void)
{
    StopWdt();       // Disable any WDT currently on.
    InitTimer0();    // Enable Timer
    InitAdc();		 // Enable ADCs
    LcdInitialize(); // Connect to LCD
    TwiMasterInit(); // Enable I2C
    sei();			 // Master interrupt bit.

    LcdClear();
    LcdCursorOnUnderline(); // Tracks the joystick position as cursor.

    MotorControlInit(); // set motor control pins for output

    while (1)
    {
        JoystickTask(); // Get joystick position every cycle
        if (sysMode == AUTO && uiState == NAVIGATING_MENUS)
        {
            AutoMotorTask();
        }
        if (adcReadFlag == true)
        {
            adcReadFlag = false;
            photocellVoltage = ReadAdc(); // Read the brightness
        }
        if (lcdUpdateFlag == true)
        {
            lcdUpdateFlag = false;
            UserInterfaceTask(); // Update the LCD
        }
    }
    return 0;
}
