# specify cross compiler
CC = arm-none-eabi-gcc 
# tool used to translate or copy object files - elf -> binary
OBJCOPY = arm-none-eabi-objcopy
# 
OBJDUMP = arm-none-eabi-objdump
# tool that will show the memory size of the sections and binary files
# used to see how much memory the compiled program will take
SIZE = arm-none-eabi-size
# native debugger
GBD = arm-none-eabi-gdb
# target processor
MACH = cortex-m4

# specify the directory of the startup file
STARTUP_DIR = .

# specify the directory of source files
SRC_DIR = .

# linker script
LINKER_SCRIPT = linker_stm32F4.ld

# output file name
TARGET = main

# flags
# enable Thumb Code - disables FPU - sets standard - disable code optimatiztion -enable compiler warning messages
CFLAGS = -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -O0 -Wall
LDFLAGS = -T$(LINKER_SCRIPT) -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=$(TARGET).map

# object files
OBJS = $(STARTUP_DIR)/startup_stm32F4.o \
    $(SRC_DIR)/main.o \
    $(SRC_DIR)/led_control.o \
	$(SRC_DIR)/syscalls.o

# declare all clean debug load flash as "fake" (isn't a file but a recipe)
# make won't check to see if these files exist
.PHONY: all inspect clean debug openocd flash

all: $(TARGET).bin

# convert final elf into raw binary
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# build your final elf and attach section sizes
$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@
	$(SIZE) $@

# pattern rule for building object files
# can use this method if you want all your object files to have the same naming convention as your input .c files
# alternative method - 
# $(STARTUP_DIR)/startup_stm32F4.o: $(STARTUP_DIR)/startup_stm32F4.c
# 	$(CC) $(CFLAGS) -c $< -o $@
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

inspect:
	$(OBJDUMP) -d $(TARGET).elf

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(TARGET).map 

debug:
	$(GDB) $(TARGET).elf -x gdbinit.cfg

# Start OpenOCD in the background (&)
# # ctrl-c to manually shutdown
openocd:
	openocd -f board/stm32f4discovery.cfg

# this is currently incorrect to flash .bin
# monitor write_image erase "$(local_path)\\$(TARGET).bin" 0x08000000
flash:
	openocd -f board/stm32f4discovery.cfg -c "program $(TARGET).elf verify reset exit"