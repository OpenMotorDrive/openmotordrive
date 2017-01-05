GCC_DIR := /opt/gcc-arm-none-eabi-4_9-2015q3
LIBOPENCM3_DIR := submodules/libopencm3
LIBCANARD_DIR := submodules/libcanard
LDSCRIPT := stm32f3.ld

ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS := --static -nostartfiles -L$(LIBOPENCM3_DIR)/lib -T$(LDSCRIPT) -Wl,--gc-sections --specs=rdimon.specs -u printf_float

LDLIBS := -lopencm3_stm32f3 -lm -Wl,--start-group -lc -lgcc -lrdimon -Wl,--end-group

CFLAGS += -std=gnu11 -O3 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Wimplicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fsingle-precision-constant -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -Isrc -I$(LIBOPENCM3_DIR)/include -I$(LIBCANARD_DIR) -DSTM32F3 -D"CANARD_ASSERT(x)"="do {} while(0)" -DGIT_HASH=0x$(shell git rev-parse --short=8 HEAD)

COMMON_OBJS := $(addprefix build/,$(addsuffix .o,$(basename $(shell find src/esc -name "*.c"))))
PROGS := $(addprefix build/bin/,$(addsuffix .elf,$(notdir $(basename $(shell ls src/programs/*.c)))))

.PHONY: all
all: $(LIBOPENCM3_DIR) $(PROGS)

build/bin/%.elf: $(COMMON_OBJS) build/canard.o build/src/programs/%.o
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(LDFLAGS) $(ARCH_FLAGS) $^ $(LDLIBS) -o $@
	@arm-none-eabi-size $@

build/%.bin: build/%.elf
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-objcopy -O binary $< $@

.PRECIOUS: build/src/%.o
build/src/%.o: src/%.c $(LIBOPENCM3_DIR)
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -S $< -o $(patsubst %.o,%.S,$@)
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@

build/canard.o: $(LIBCANARD_DIR)/canard.c
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(CFLAGS) $(ARCH_FLAGS) -c $< -o $@

.PHONY: $(LIBOPENCM3_DIR)
$(LIBOPENCM3_DIR):
	@echo "### BUILDING $@"
	@$(MAKE) -C $(LIBOPENCM3_DIR)

.PHONY: %-upload
%-upload: build/bin/%.elf
	@echo "### UPLOADING"
	@openocd -f openocd.cfg -c "program $< verify reset exit"

.PHONY: %
%: build/bin/%.elf ;

.PHONY: clean
clean:
	@$(MAKE) -C $(LIBOPENCM3_DIR) clean
	@rm -rf build
