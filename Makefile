CC = avr-gcc
MCU_DEFINE = -mmcu=atmega328pb

all: flash

solarthermal_control:
	$(CC) $(MCU_DEFINE) solarthermal_control.c lcd_lib.c -o solarthermal_control.bin

flash: solarthermal_control
	avrdude -p m328pb -c atmelice_isp -P usb -U flash:w:solarthermal.bin:i

clean:
	rm -f solarthermal_control.bin