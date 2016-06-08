.PHONY: all clean

GCC_DIR := /opt/gcc-arm-none-eabi-4_9-2015q3
LIBOPENCM3_DIR := libopencm3
LDSCRIPT := stm32f3.ld

BIN_NAME := main

ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS := --static -nostartfiles -L$(LIBOPENCM3_DIR)/lib -T$(LDSCRIPT) -Wl,-Map=$(BIN_NAME).map -Wl,--gc-sections

LDLIBS := -lopencm3_stm32f3 -lm -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

CFLAGS := -Os -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -I$(LIBOPENCM3_DIR)/include -DSTM32F3

OBJECTS := main.o init.o pwm.o timing.o helpers.o encoder.o drv.o adc.o serial.o curr_pid.o motor.o ringbuf.o param.o serial_protocol.o

all: main.elf pre-build

pre-build:
	+make -C libopencm3

main.elf main.map: $(OBJECTS)
	arm-none-eabi-gcc $(LDFLAGS) $(ARCH_FLAGS) $^ $(LDLIBS) -o $(BIN_NAME).elf


%.o: %.c pre-build
	arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $<

main.bin: main.elf
	arm-none-eabi-objcopy -O binary main.elf main.bin

upload: main.bin
	st-flash write main.bin 0x8000000

clean:
	+make -C libopencm3 clean
	rm -f *.d *.o *.elf *.map *.bin
