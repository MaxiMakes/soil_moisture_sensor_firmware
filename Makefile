OBJS=main.o
ELF=$(notdir $(CURDIR)).elf  
HEX=$(notdir $(CURDIR)).hex
F_CPU=20000000L/6
#F_CPU=20000000L
#F_CPU=32768L

CFLAGS=-mmcu=attiny402  -Os -Wall
CFLAGS+=-I /usr/avr/include/ -DF_CPU=$(F_CPU)
LDFLAGS=-mmcu=attiny402
CC=avr-gcc
LD=avr-gcc

all:    $(HEX)  

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(HEX): $(ELF)
	avr-objcopy -O ihex -R .eeprom $< $@

flash:  $(HEX)
	pyupdi -d tiny402 -c /dev/tty.usbserial-FTF5HUAV -f $(HEX)

read-fuses:
	pyupdi -d tiny402 -c /dev/tty.usbserial-FTF5HUAV -fr

clean:
	rm -rf $(OBJS) $(ELF) $(HEX)