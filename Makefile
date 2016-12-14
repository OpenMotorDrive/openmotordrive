GCC_DIR := /opt/gcc-arm-none-eabi-4_9-2015q3
LIBOPENCM3_DIR := submodules/libopencm3
LIBCANARD_DIR := submodules/libcanard
LDSCRIPT := stm32f3.ld

ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS := --static -nostartfiles -L$(LIBOPENCM3_DIR)/lib -T$(LDSCRIPT) -Wl,-Map=build/main.map -Wl,--gc-sections --specs=rdimon.specs -u printf_float

LDLIBS := -lopencm3_stm32f3 -lm -Wl,--start-group -lc -lgcc -lrdimon -Wl,--end-group

CFLAGS += -std=gnu11 -O3 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -Isrc -I$(LIBOPENCM3_DIR)/include -I$(LIBCANARD_DIR) -DSTM32F3

OBJS := main.o esc/programs.o programs/servo_test.o programs/spin_test.o programs/print_input_voltage.o programs/print_encoder.o programs/phase_output_test.o programs/can_servo.o programs/can_encoder.o esc/init.o esc/pwm.o esc/timing.o esc/helpers.o esc/encoder.o esc/drv.o esc/adc.o esc/serial.o esc/curr_pid.o esc/motor.o esc/ringbuf.o esc/param.o esc/can.o esc/canbus_debug.o esc/semihost_debug.o
OBJS := $(addprefix build/,$(OBJS))

.PHONY: all
all: $(LIBOPENCM3_DIR) build/main.elf

.PHONY: $(LIBOPENCM3_DIR)
$(LIBOPENCM3_DIR):
	@echo "### BUILDING $@"
	@$(MAKE) -C $(LIBOPENCM3_DIR)

build/main.elf build/main.map: $(OBJS) build/canard.o
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	arm-none-eabi-gcc $(LDFLAGS) $(ARCH_FLAGS) $^ $(LDLIBS) -o build/main.elf

-include $(OBJS:.o=.d)
build/%.o: src/%.c $(LIBOPENCM3_DIR)
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@

build/canard.o: $(LIBCANARD_DIR)/canard.c
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@

build/main.bin: build/main.elf
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-objcopy -O binary build/main.elf build/main.bin

.PHONY: upload
upload: build/main.bin
	@echo "### UPLOADING"
	@st-flash write build/main.bin 0x8000000

.PHONY: clean
clean:
	@$(MAKE) -C $(LIBOPENCM3_DIR) clean
	@rm -rf build
