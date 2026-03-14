CC = avr-gcc
MCU_DEFINE = -mmcu=atmega328pb

solarthermal_control:
	$(CC) $(MCU_DEFINE) solarthermal_control.c lcd_lib.c -o solarthermal_control.bin

clean:
	rm -f solarthermal_control.bin