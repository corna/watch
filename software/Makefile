MMCU=attiny85
CPUCLOCK=8000000L
FILENAME=main

PROGRAMMER=usbasp
LANSTD=gnu11
OPTIMIZATION=Os
WARNINGS=Wall
REMOTE_SPI_DEVICE=/dev/spidev0.0

LFUSE=0xc2
HFUSE=0xff
EFUSE=0xff

ADDITIONAL_ARGS=
ADDITIONAL_SOURCES=USI_TWI_Master/USI_TWI_Master.c ws2812/ws2812_avr.c veml7700/veml7700.c i2c.c

all:
	avr-gcc -$(OPTIMIZATION) -$(WARNINGS) -std=$(LANSTD) -mmcu=$(MMCU) -D F_CPU=$(CPUCLOCK) $(ADDITIONAL_ARGS) -I ./ -o $(FILENAME).o $(FILENAME).c $(ADDITIONAL_SOURCES)
	avr-objcopy -R .eeprom -O ihex $(FILENAME).o $(FILENAME).hex
	@avr-size -C --mcu=$(MMCU) $(FILENAME).o | egrep "Program|Data"

flash:
	avrdude -c $(PROGRAMMER) -p $(MMCU) -P USB -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m -U flash:w:$(FILENAME).hex:i

flashall: all flash

clean:
	@rm -f *.o *.hex
