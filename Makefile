# ARGUMENTS
COMPILER = avr-gcc
MCU = atmega328p
FLAGS = -Wall -mmcu=atmega328p -DF_CPU=8000000
DIRS = . ./FreeRTOS-Kernel/include/ ./FreeRTOS-Kernel/portable/GCC/ATMega328p/ ./LCD/
INCLUDE = $(foreach dir, $(DIRS), -I$(dir))
SOURCE_FILES = ./FreeRTOS-Kernel/portable/GCC/ATMega328p/port.c ./FreeRTOS-Kernel/list.c ./FreeRTOS-Kernel/queue.c ./FreeRTOS-Kernel/tasks.c main.c ./LCD/lcd_lib.c
SOURCES = $(foreach source, $(SOURCE_FILES), $(source))
OBJECT_FILES = main.o port.o list.o queue.o tasks.o lcd_lib.o
OBJECTS = $(foreach obj, $(OBJECT_FILES), $(obj))

# TARGETS
all:compile
	$(COMPILER) $(FLAGS) $(INCLUDE) -o main.elf $(OBJECTS)
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(MCU) main.elf

dump:all
	avr-objdump -d main.elf > dump.txt

compile:
	$(COMPILER) $(FLAGS) $(INCLUDE) -c $(SOURCES)

flash:all
	avrdude -c arduino -p $(MCU) -P /dev/ttyACM0 -U flash:w:main.hex:i

clean:
	rm -f *.o *.hex *.elf