.PHONY: all clean upload libopencm3

GCC_DIR := /opt/gcc-arm-none-eabi-4_9-2015q3
LIBOPENCM3_DIR := libopencm3
LDSCRIPT := stm32f3.ld

ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS := --static -nostartfiles -L$(LIBOPENCM3_DIR)/lib -T$(LDSCRIPT) -Wl,-Map=build/main.map -Wl,--gc-sections

LDLIBS := -lopencm3_stm32f3 -lm -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

CFLAGS += -O3 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -I$(LIBOPENCM3_DIR)/include -DSTM32F3

OBJECTS := main.o init.o pwm.o timing.o helpers.o encoder.o drv.o adc.o serial.o curr_pid.o motor.o ringbuf.o param.o can.o

all: build/main.elf libopencm3

libopencm3:
	@echo "### BUILDING $@"
	@$(MAKE) -C libopencm3

build/main.elf build/main.map: $(addprefix build/,$(OBJECTS))
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(LDFLAGS) $(ARCH_FLAGS) $^ $(LDLIBS) -o build/main.elf


build/%.o: src/%.c libopencm3
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@

build/main.bin: build/main.elf
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-objcopy -O binary build/main.elf build/main.bin

upload: build/main.bin
	@echo "### UPLOADING"
	@st-flash write build/main.bin 0x8000000

clean:
	@$(MAKE) -C libopencm3 clean
	@rm -rf build
