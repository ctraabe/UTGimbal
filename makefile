# Note that without an argument, make simply tries to build the first target
# (not rule), which in this case is $(HEX)

CC      = avr-gcc
CP      = avr-objcopy
DUDE    = avrdude

MCU     = atmega168
F_CPU   = 16000000L
BAUD    = 19200

LFLAGS  = -std=gnu99 -Ofast -mmcu=$(MCU) -Wall -pedantic -Wstrict-prototypes \
          -Wundef -Werror -fshort-enums -ffreestanding -Wl,--relax
CFLAGS  = -c -g $(LFLAGS) -Wa,-adhlns=$(<:%.c=%.lst) -DF_CPU="$(F_CPU)"
SOURCES = $(wildcard *.c)
DEPENDS = $(SOURCES:.c=.d)
OBJECTS = $(SOURCES:.c=.o)
ASSEMBL = $(SOURCES:.c=.lst)

ELF     = build.elf
HEX     = build.hex

# Rule to make dependency "makefiles"
%.d: %.c
	$(CC) -mmcu=$(MCU) -MM -MT '$(<:.c=.o) $@' $< -MF $@

# Rule to make the compiled objects
%.o: %.c %.d
	$(CC) $(CFLAGS) -o $@ $<

# Declare targets that are not files
.PHONY: install clean

# Target to build the .hex file
$(HEX): $(ELF)
	$(CP) -O ihex $(ELF) $(HEX)

# Target to build the .elf file
# NOTE: -lm includes the math library (libm.a)
$(ELF): $(OBJECTS)
	$(CC) $(LFLAGS) -lm -o $(ELF) $(OBJECTS)

# Include the dependency "makefiles"
-include $(DEPENDS)

# Target to program the board
install: $(HEX)
	$(DUDE) -C/usr/share/arduino/hardware/tools/avrdude.conf -p$(MCU) \
	-carduino -P/dev/ttyUSB0 -b$(BAUD) -D -Uflash:w:$(HEX):i 

# Target to clean up the directory (leaving only source)
clean:
	rm -f $(HEX) $(ELF) $(OBJECTS) $(DEPENDS) $(ASSEMBL)
