TARGET := ut-gimbal

# MCU     = atmega168
MCU    := atmega328p
F_CPU  := 16000000L

CFLAGS  = -c -g -DF_CPU="$(F_CPU)" $(LFLAGS) \
          -Wa,-adhlns=$(addprefix $(BUILD_PATH)/,$(<:%.c=%.lst))
LFLAGS := -std=gnu99 -Ofast -mmcu=$(MCU) -pedantic -Werror -Wall -Wextra \
          -Wstrict-prototypes -Wundef -fshort-enums -ffreestanding -Wl,--relax

CC     := avr-gcc
CP     := avr-objcopy
DUDE   := avrdude

ifeq ($(MCU),atmega168)
        BAUD := 19200
else
        BAUD := 57600
endif

# If the environment variable DEV_BUILD_PATH is set, then the build files will
# be placed there in a named sub-folder, otherwise a build directory will be
# created in the current directory
ifneq ($(DEV_BUILD_PATH),)
BUILD_PATH := $(DEV_BUILD_PATH)/build/$(TARGET)
else
BUILD_PATH := build
endif

SOURCES = $(wildcard *.c)
DEPENDS = $(addprefix $(BUILD_PATH)/, $(SOURCES:.c=.d))
OBJECTS = $(addprefix $(BUILD_PATH)/, $(SOURCES:.c=.o))
ASSEMBL = $(addprefix $(BUILD_PATH)/, $(SOURCES:.c=.lst))

ELF    := $(BUILD_PATH)/$(TARGET).elf
HEX    := $(BUILD_PATH)/$(TARGET).hex

# Rule to make dependency "makefiles"
$(BUILD_PATH)/%.d: %.c
	mkdir -p $(BUILD_PATH)
	$(CC) -mmcu=$(MCU) -MM -MT '$(addprefix $(BUILD_PATH)/, $(<:.c=.o)) $@' $< -MF $@

# Rule to make the compiled objects
$(BUILD_PATH)/%.o: %.c $(BUILD_PATH)/%.d
	$(CC) $(CFLAGS) -o $@ $<

# Declare targets that are not files
.PHONY: program clean


# Note that without an argument, make simply tries to build the first target
# (not rule), which in this case is this target to build the .hex
$(HEX): $(ELF)
	$(CP) -O ihex $(ELF) $(HEX)

# Target to build the .elf file
# NOTE: -lm includes the math library (libm.a)
$(ELF): $(OBJECTS)
	$(CC) $(LFLAGS) -o $(ELF) $(OBJECTS) -lm

# Include the dependency "makefiles"
ifneq ($(MAKECMDGOALS),clean)
-include $(DEPENDS)
endif

# Target to program the board
program: $(HEX)
	$(DUDE) -C/usr/share/arduino/hardware/tools/avrdude.conf -p$(MCU) \
	-carduino -P/dev/ttyUSB0 -b$(BAUD) -D -Uflash:w:$(HEX):i

# Target to clean up the directory (leaving only source)
clean:
	rm -f $(HEX) $(ELF) $(OBJECTS) $(DEPENDS) $(ASSEMBL)
	rmdir $(BUILD_PATH)
