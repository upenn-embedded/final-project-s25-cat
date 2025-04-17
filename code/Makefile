MCU = atmega328p
F_CPU = 16000000UL
CC = /c/avr8-gnu-toolchain/bin/avr-gcc.exe
OBJCOPY = /c/avr8-gnu-toolchain/bin/avr-objcopy.exe

CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall
SRC = main.c motor_control.c spi.c uart.c ultrasonic.c
TARGET = main

all: $(TARGET).hex

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f *.o *.elf *.hex
