# solarthermal_control

## Background

This program was a student project I wrote for my Microprocessor Lab course in undergrad. I really enjoyed that class, and realized that I wanted very much to work on embedded systems in my future.

At the time, I was living and working on a student-run farm at the university, where I had taken a special interest in the solar thermal array and greenhouse. A problem we frequently had on the farm was that the greenhouse was getting much warmer than needed, so we often covered the solar thermal array with a blanket to prevent more heat absorpbtion. I imagined it would be far better if we could have the blanket on a furling system with a motor, such that some basic control system could be implemented to automatically manage the solar energy input into the system.

Well, I never did implement that system, but I did make a small demo of the idea for Microprocessor Lab. This project is that demo.

## Construction

The physical system was very simple, being comprised of a shallow cardboard box inclined at an angle, a lid which could be pulled up or slid down to open/close the box, and a motor mounted on a tower with a pulley controlling the height of the lid. Mounted inside the box were four photoresistors, equally spaced along the height of the box.

The remaining electronics were off to the side of the box, and were the microcontroller, a stepper motor driver board, a joystick, a 16x2 LCD, and four resistors pairing with the photoresistors to form voltage dividers.

## Hardware Used

|Component|Function|Interface|
|---------|--------|---------|
|AVR Xplained Mini|ATmega328PB Microcontroller including ADCs and communication peripherals|N/A|
|SparkFun Qwiic Joystick|User input to system|I2C|
|Newhaven Display 16x2 LCD|System output to user|UART|
|28BY-J-48 Stepper Motor|Drives pulley to control shade|ON/OFF excitement of coils|
|ULN2003 Stepper Motor Driver Board|Supplies adequate power to stepper|N/A|
|4x 10K Photoresistors and 4x 1K Resistors|Used together as voltage divider to measure brightness as voltage between ~0V and ~5V|N/A|
|2x 10K Resistors|Pull-up resistors for I2C bus|N/A|

## Operation

There are two modes of operation: Auto and Manual. In Auto, the user uses the joystick to set a voltage setpoint, and the control loop rotates the stepper motor until the photocells have an average voltage within 0.1V of the setpoint. In Manual, the user directly controls the movement of the stepper motor through the joystick. At all times the current voltage, setpoint, and operational mode are displayed on screen, while the user navigates the system using the joystick.

## Build Dependencies

### List

- make
- avr-gcc
- avr-libc
- avrdude

### Acquiring Dependencies

`$ apt install make gcc-avr avr-libc avrdude`

## Build Process

The included Makefile provides targets to build the binary and flash the board in `solarthermal_control` and `flash` respectively.

 `$ make all` will build the binary then flash over USB.