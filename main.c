// ###############################################
// # solarthermal_control.c
// #
// # POC for limiting solar input to solarthermal
// # array at Spring Valley Student Farm.
// #
// # Zachary Stone, November 2021
// ###############################################

// INCLUDES
#include <avr/io.h>
//#include <avr/interrupt.h>
#include "lcd_lib.h"
//#include <avr/wdt.h>
#include <stdbool.h>
#include <stdlib.h> // contains dtostrf();
//#include <avr/pgmspace.h>
#include <math.h>
//#include <stdint.h>
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// DEFINES
// We can use the same priority as the idle task, a define controls
// how idle and this task collide.
#define mainLCD_UPDATE_TASK_PRIORITY  (tskIDLE_PRIORITY)
#define mainJOYSTICK_TASK_PRIORITY    (tskIDLE_PRIORITY+1)
#define mainADC_READ_TASK_PRIORITY    (tskIDLE_PRIORITY+2)
#define mainRUN_MOTOR_TASK_PRIORITY   (tskIDLE_PRIORITY+3) // higher is greater priority
// I2C communication
#define JOY_ADDR  0x20 // i2c addr
#define X_REG     0x03 // i2c registers
#define Y_REG     0x05 // i2c registers
#define CLICK_REG 0x07 // i2c registers
#define WRITE_BIT 0    // i2c control
#define READ_BIT  1    // i2c control
// Motor Control
#define STEP_CW   1 // motor directions
#define STEP_CCW  2 // motor directions
#define IN1 PORTD0  // motor step pins
#define IN2 PORTD2  // motor step pins
#define IN3 PORTD3  // motor step pins
#define IN4 PORTD4  // motor step pins
// Photosensor Related
#define ERR_TOL 0.1 // photosensor voltage tolerance
// User Interface
#define NAVIGATING_MENUS    0 // Values for uiState
#define ADJ_SHADE_POS       1 // Values for uiState
#define ADJ_BRIGHT_SETPOINT 2 // Values for uiState
#define POSITIVE_LEAN 192 // joystick sensitivity
#define NEGATIVE_LEAN  64 // joystick sensitivity
#define JOY_CLICKED     0 // joystick toggle
#define JOY_DEBOUNCE  pdMS_TO_TICKS(250) // Joystick read timer
// System Mode
#define SYS_MODE_AUTO    0
#define SYS_MODE_MANUAL  1
// LCD
#define MAX_HORIZ_POS  15 // LCD position limit
#define MAX_VERT_POS    1 // LCD position limit

// DATA TYPES
struct CursorPos {
    uint8_t x;
    uint8_t y;
};

// GLOBALS
uint8_t sysMode = SYS_MODE_AUTO; // The overall control mode
uint8_t uiState = NAVIGATING_MENUS; // The state of the ui
const struct CursorPos cursorControlModePosition = {7,1};
const struct CursorPos cursorAdjustPosition = {9,0};
uint8_t lcdPosX = 0;
uint8_t lcdPosY = 0;
float photocellVoltage;
uint8_t voltageSetpoint = 0;
uint16_t debounceTime;
uint8_t ucManualRun = 0;
// Resource Protection
StaticSemaphore_t xPhotocellVoltageMutexBuffer;
SemaphoreHandle_t xPhotocellVoltageMutex;
// All text used for display
const unsigned char voltageMsg[] = "V: ";
const unsigned char controlMsg[] = "Cntrl: ";
const unsigned char setpointMsg[] = "Set: ";
const unsigned char adjMsg[] = "Adjust";
const unsigned char motorAdjMsgOne[] = "Adj shade with";
const unsigned char motorAdjMsgTwo[] = "joystick.";

// // StopWdt disables any already-configured watchdog
// void StopWdt(void)
// {
//     wdt_reset();                    // Reset watchdog timer
//     MCUSR &= ~_BV(WDRF);            // Shut off watchdog reset flag
//     WDTCSR |= _BV(WDCE) | _BV(WDE); // Watchdog change enable and watchdog enable
//     WDTCSR = 0x00;                  // Disable watchdog
// }

// // InitTimer0 sets TC0 for 500us resolution.
// void InitTimer0(void)
// {
//     TCCR0A |= _BV(WGM01); // Set TC0 to CTC mode
//     OCR0A = 124;          // Count up to 124
//     TIMSK0 = _BV(OCIE0A); // Enable timer 0 compare A ISR
//     TCCR0B = 3;           // Set scaling to divide by 64 counts and start timer.
// }

// // TC0 interrupt. Executes every 500us.
// ISR(TIMER0_COMPA_vect)
// {
//     if (adcReadTimer > 0)
//     {
//         adcReadTimer--;
//     }
//     else
//     {
//         adcReadFlag = true;   // I ended up grouping these two on the same timer
//         lcdUpdateFlag = true;
//         adcReadTimer = ADC_DELAY;
//     }
//     if (debounceTime > 0)
//     {
//         debounceTime--;       // This is to control how often the joystick position is checked.
//     }                         // Otherwise the joystick provides input at full cycle speed.
// }

// MotorControlInit sets the motor control pins for output
void MotorControlInit(void)
{
    // Set data direction register for output on motor control pins
    DDRD = (1<<DDD0 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4);
}

// StepMotor takes a direction input and steps once in that direction
void StepMotor(int direction)
{
    // Persistent step index
    static int8_t stepIndex = 0;
    // Successive values for PORTD to step the motor
    static const uint8_t stepTable[4] = {_BV(IN1),_BV(IN2),_BV(IN3),_BV(IN4)};

    stepIndex += direction;

    // Handle looping of stepTable
    if (stepIndex > 3) stepIndex = 0;
    else if (stepIndex < 0) stepIndex = 3;
    // Zero relevant bits before setting, then set the output
    PORTD &= ~(_BV(IN1)|_BV(IN2)|_BV(IN3)|_BV(IN4));
    PORTD |= stepTable[stepIndex];
}

// vMotorTask steps the motor
void vMotorTask(void* pvParameters)
{
    // Functional
    float flCellVoltage;
    float flErrSig;
    // Timing
    TickType_t xLastWakeTime;
    const TickType_t xTicksToGo = pdMS_TO_TICKS(10); // run the motor on a 10ms period
    xLastWakeTime = xTaskGetTickCount();
    // Recurrent execution
    for (;;)
    {
        if (sysMode == SYS_MODE_MANUAL)
        {
            // Reading ucManualRun is atomic. No need for mutex.
            if (ucManualRun == STEP_CCW) StepMotor(STEP_CCW);
            else if (ucManualRun == STEP_CW) StepMotor(STEP_CW);
        }
        else if (sysMode == SYS_MODE_AUTO)
        {
            // Retrieve the current photocell voltage and setpoint
            if (xPhotocellVoltageMutex != NULL)
            {
                if (xSemaphoreTake(xPhotocellVoltageMutex, 0) == pdPASS)
                {
                    // Read of photocellVoltage is NOT atomic
                    flCellVoltage = photocellVoltage;
                    xSemaphoreGive(xPhotocellVoltageMutex); // Call shouldn't fail

                    // Compute the error signal and adjust run the motor
                    // Photocells are being used in a voltage divider setup such that more light
                    // creates a higher measured voltage.
                    flErrSig = voltageSetpoint - flCellVoltage; // Compute difference.
                    if (fabs(flErrSig) > ERR_TOL)
                    {
                        if (flErrSig > 0) StepMotor(STEP_CCW); // not enough light, open the shutter
                        else if (flErrSig < 0) StepMotor(STEP_CW); // too much light, close the shutter
                    }
                }
            }
            else
            {
                // nothing, mutex does not exist for some reason
            }
        } // end sysMode == SYS_MODE_AUTO

        xTaskDelayUntil(&xLastWakeTime, xTicksToGo);
    } // end for(;;)
}

// TwiMasterInit configures registers for I2C operation
void TwiMasterInit(void)
{
    TWSR &= ~(_BV(TWPS1) | _BV(TWPS0)); // ensure bits for prescaler=1
    TWBR = 42; 	       // 8MHz clock; prescaler=1; SCLK=80KHz
    TWDR = 0xFF;       // Default data content, SDA released
    TWCR = _BV(TWEN);  // Enable TWI & Acknowledgments.
}

// TwiRead tells the target at Address which Data we want, then reads and returns the data
int8_t TwiRead(uint8_t Address, uint8_t Data)
{
    // First we must write to target to tell it which data we want
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); // Send the START condition
    while(!(TWCR & _BV(TWINT)));                // Wait for TWINT to set

    TWDR = (Address<<1) | (WRITE_BIT); // Load in the address and write bit
    TWCR = _BV(TWINT) | _BV(TWEN); // clear the interrupt to begin transmission
    while(!(TWCR & _BV(TWINT)));   // wait for TWINT to set

    TWDR = Data;					// Load data in
    TWCR = _BV(TWINT) | _BV(TWEN);  // clear interrupt
    while(!(TWCR & _BV(TWINT)));    // wait for twint to set

    // Instead of stopping here, we now begin a read

    // Now read the data
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);  // send another start condition
    while(!(TWCR & _BV(TWINT)));				 // wait for TWINT

    TWDR = (Address<<1) | (READ_BIT);           // load in the address and read bit
    TWCR = _BV(TWINT) | _BV(TWEN);               // clear twint
    while(!(TWCR & _BV(TWINT)));                 // wait for twint

    TWCR = _BV(TWINT) | _BV(TWEN);				 // clear twint
    while (!(TWCR & _BV(TWINT)));				 // wait for twint
    uint8_t readOut = TWDR;					 // read the received value

    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);  // transmit STOP
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
void vJoystickTask(void* pvParams)
{
    // Functional
    uint8_t joyHoriz;
    uint8_t joyVert;
    uint8_t click;
    uint8_t tempX;
    uint8_t tempY;
    // Timing
    TickType_t xLastWakeTime;
    const TickType_t xTicksToGo = pdMS_TO_TICKS(50); // Check the joystick on a 50ms period
    xLastWakeTime = xTaskGetTickCount();
    // Recurrent Execution
    for(;;)
    {
        // Reading in X,Y, and click of joystick.
        joyHoriz = TwiRead(JOY_ADDR, X_REG);
        joyVert = TwiRead(JOY_ADDR, Y_REG);
        click = TwiRead(JOY_ADDR, CLICK_REG);
        // Save the position of the cursor from the previous call
        tempX = lcdPosX;
        tempY = lcdPosY;
        if (debounceTime >= xTicksToGo)
        {
            debounceTime -= xTicksToGo;
        }

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
                            sysMode = (sysMode == SYS_MODE_AUTO) ? SYS_MODE_MANUAL : SYS_MODE_AUTO; // Change to other mode
                            LcdClear();
                        }
                        else if (CheckCursorPos(cursorAdjustPosition))
                        {
                            uiState = (sysMode == SYS_MODE_AUTO) ? ADJ_BRIGHT_SETPOINT : ADJ_SHADE_POS;
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
                // Joystick controls signals motor to run
                else
                {
                    // Assignment to 8bit ucManualRun is atomic. No need to mutex.
                    // if joystick up, curtain up
                    if (joyVert > POSITIVE_LEAN) ucManualRun = STEP_CCW;
                    // if joystick down, curtain down
                    else if (joyVert < NEGATIVE_LEAN) ucManualRun = STEP_CW;
                    else ucManualRun = 0;
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

        xTaskDelayUntil(&xLastWakeTime, xTicksToGo);
    } // end for(;;)
}

// InitAdc initializes the on board ADC
void InitAdc(void)
{
    // ON PC4
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    // ADEN enables ADC
    // ADPS is prescaler -- 111 is div by 128.
    ADCSRA |= _BV(ADSC); // begins the conversion
}

// ReadAdcChannel reads the ADC given by input channel
uint16_t ReadAdcChannel(uint8_t channel)
{
    ADMUX = channel;            // specify which ADC to look at
    ADCSRA |= _BV(ADSC);        // begin a read
    while (ADCSRA & _BV(ADSC)); // ADSC clears when read is complete. Should be 13 cycles(812ns)
    return ADC;
}

// ReadAdcTask reads the four photocells and returns the average in volts
void vReadAdcTask(void* pvParameters)
{
    // Functional
    uint16_t sum = 0; // ADC has 10 bit precision
    uint8_t i;
    float avg;
    // Timing
    TickType_t xLastWakeTime;
    const TickType_t xTicksToGo = pdMS_TO_TICKS(5); // Read the voltage on a 5ms period
    xLastWakeTime = xTaskGetTickCount();
    // Recurrent execution
    for (;;)
    {
        sum = 0;
        avg = 0;
        // Sum the reading across 4 photocells
        for (i = 0; i < 4; i++)
        {
            sum += ReadAdcChannel(i);
        }
        avg = sum/4.00;
        // Save the average in terms of voltage across cells
        if (xPhotocellVoltageMutex != NULL)
        {
            if (xSemaphoreTake(xPhotocellVoltageMutex, 0) == pdPASS)
            {
                // Assignment to float is NOT atomic.
                photocellVoltage = ((float)avg/1023.00)*5.00;
                xSemaphoreGive(xPhotocellVoltageMutex);
            }
        }
        else
        {
            // mutex did not exist
        }

        xTaskDelayUntil(&xLastWakeTime, xTicksToGo);
    } // end for(;;)
}

// LcdUpdateTask operates a state machine to display all needed prompts on the LCD
void vLcdUpdateTask(void* pvParameters)
{
    // Functional
    char charBuf[4];
    // Timing
    TickType_t xLastWakeTime;
    const TickType_t xTicksToGo = pdMS_TO_TICKS(50); // Update the LCD on a 50ms period
    xLastWakeTime = xTaskGetTickCount();
    // Recurrent Execution
    for (;;)
    {
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
                if (sysMode == SYS_MODE_MANUAL)
                {
                    //################
                    //V:x.xxx   Adjust
                    //Cntrl: Manual
                    //################
                    LcdFlashString(adjMsg,9,0);
                    LcdFlashString(controlMsg,0,1);
                    LcdString("Manual",6);
                }
                else if (sysMode == SYS_MODE_AUTO)
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

        xTaskDelayUntil(&xLastWakeTime, xTicksToGo);
    } // end for(;;)
}

void vSetupPrimaryLed(void)
{
    DDRB |= _BV(PB5);
    // Make sure it's off to begin
    PORTB &= ~(_BV(PB5));
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    // just turn on the primary LED
    PORTB |= _BV(PB5);
}

void vApplicationIdleHook(void)
{
    //nothing;
}

void vSetupMutexes(void)
{
    // Create each of the mutexes used in the system
    xPhotocellVoltageMutex = xSemaphoreCreateMutexStatic(&xPhotocellVoltageMutexBuffer);
}

// Entry point of program
int main(void)
{
    // Task Setup Vars
    static StaticTask_t xMotorTaskControlBuffer;
    static StackType_t axMotorTaskStack[configMINIMAL_STACK_SIZE];
    static TaskHandle_t xAutoMotorTaskHandle = NULL;
    static StaticTask_t xReadAdcTaskControlBuffer;
    static StackType_t axReadAdcTaskStack[configMINIMAL_STACK_SIZE];
    static TaskHandle_t xReadAdcTaskHandle = NULL;
    static StaticTask_t xJoystickTaskControlBuffer;
    static StackType_t axJoystickTaskStack[configMINIMAL_STACK_SIZE];
    static TaskHandle_t xJoystickTaskHandle = NULL;
    static StaticTask_t xLcdUpdateTaskControlBuffer;
    static StackType_t axLcdUpdateTaskStack[configMINIMAL_STACK_SIZE];
    static TaskHandle_t xLcdUpdateTaskHandle = NULL;

    // One time setup functions
    vSetupPrimaryLed(); // We will use the primary LED as a stack overflow signal
    InitAdc();		 // Enable ADCs
    LcdInitialize(); // Connect to LCD
    TwiMasterInit(); // Enable I2C
    LcdClear();
    LcdCursorOnUnderline(); // Tracks the joystick position as cursor.
    MotorControlInit(); // set motor control pins for output
    vSetupMutexes(); // Establish the mutexes
    
    xAutoMotorTaskHandle = xTaskCreateStatic(
        vMotorTask,
        "MOTOR",
        sizeof(axMotorTaskStack),
        NULL,
        mainRUN_MOTOR_TASK_PRIORITY,
        &axMotorTaskStack[0],
        &xMotorTaskControlBuffer
    );
    xReadAdcTaskHandle = xTaskCreateStatic(
        vReadAdcTask,
        "ADC",
        sizeof(axReadAdcTaskStack),
        NULL,
        mainADC_READ_TASK_PRIORITY,
        &axReadAdcTaskStack[0],
        &xReadAdcTaskControlBuffer
    );
    xJoystickTaskHandle = xTaskCreateStatic(
        vJoystickTask,
        "JOY",
        sizeof(axJoystickTaskStack),
        NULL,
        mainJOYSTICK_TASK_PRIORITY,
        &axJoystickTaskStack[0],
        &xJoystickTaskControlBuffer
    );
    xLcdUpdateTaskHandle = xTaskCreateStatic(
        vLcdUpdateTask,
        "LCD",
        sizeof(axLcdUpdateTaskStack),
        NULL,
        mainLCD_UPDATE_TASK_PRIORITY,
        &axLcdUpdateTaskStack[0],
        &xLcdUpdateTaskControlBuffer
    );

    vTaskStartScheduler();

    // Should never return
    return 0;
}
