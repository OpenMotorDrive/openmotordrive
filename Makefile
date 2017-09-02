COMMON_DIR := modules/omd_common
BOOTLOADER_DIR := modules/bootloader
LIBOPENCM3_DIR := modules/libopencm3
LIBCANARD_DIR := modules/libcanard
LDSCRIPT := $(COMMON_DIR)/configs/stm32f302k8/app.ld
BL_LDSCRIPT := $(COMMON_DIR)/configs/stm32f302k8/bl.ld
BOARD_CONFIG_HEADER := boards/org.openmotordrive.f3_ref_1.0/board.h

ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS := --static -nostartfiles -L$(LIBOPENCM3_DIR)/lib -L $(dir $(LDSCRIPT)) -T$(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs -u printf_float -Wl,--no-wchar-size-warning

LDLIBS := -lopencm3_stm32f3 -lm -Wl,--start-group -lc -lgcc -lrdimon -Wl,--end-group

CFLAGS += -std=gnu11 -O3 -ffast-math -g -Wdouble-promotion -Wextra -Wshadow -Werror=implicit-function-declaration -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes -fsingle-precision-constant -fno-common -ffunction-sections -fdata-sections -MD -Wall -Wundef -Isrc -I$(LIBOPENCM3_DIR)/include -I$(LIBCANARD_DIR) -I$(BOOTLOADER_DIR)/include -I$(COMMON_DIR)/include -DSTM32F3 -D"CANARD_ASSERT(x)"="do {} while(0)" -DGIT_HASH=0x$(shell git rev-parse --short=8 HEAD) -fshort-wchar -include $(BOARD_CONFIG_HEADER)

COMMON_OBJS := $(addprefix build/,$(addsuffix .o,$(basename $(shell find src -name "*.c"))))
COMMON_OBJS += $(addprefix build/,$(addsuffix .o,$(basename $(shell find $(COMMON_DIR)/src -name "*.c"))))

BIN := build/bin/main.elf build/bin/main.bin

.PHONY: all
all: $(LIBOPENCM3_DIR) $(BIN)

build/bin/%.elf: $(COMMON_OBJS) build/canard.o
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-gcc $(LDFLAGS) $(ARCH_FLAGS) $^ $(LDLIBS) -o $@
	@arm-none-eabi-size $@

build/bin/%.bin: build/bin/%.elf
	@echo "### BUILDING $@"
	@mkdir -p "$(dir $@)"
	@arm-none-eabi-objcopy -O binary $< $@
	@python $(COMMON_DIR)/tools/crc_binary.py $@ $@

.PRECIOUS: build/%.o
build/%.o: %.c $(LIBOPENCM3_DIR)
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
	@$(MAKE) -C $(LIBOPENCM3_DIR) CFLAGS="-fshort-wchar"

upload: build/bin/main.elf build/bin/main.bin
	@echo "### UPLOADING"
	@openocd -f openocd.cfg -c "program $< verify reset exit"

.PHONY: clean
clean:
	@$(MAKE) -C $(LIBOPENCM3_DIR) clean
	@rm -rf build

.PHONY: bootloader
bootloader:
	@echo "### BUILDING BOOTLOADER..."
	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_HEADER=$(abspath $(BOARD_CONFIG_HEADER)) clean
	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_HEADER=$(abspath $(BOARD_CONFIG_HEADER)) USE_LTO=1

.PHONY: bootloader-upload
bootloader-upload: bootloader
	@echo "### UPLOADING BOOTLOADER..."
	@$(MAKE) -C $(BOOTLOADER_DIR) LDSCRIPT=$(abspath $(BL_LDSCRIPT)) BOARD_CONFIG_HEADER=$(abspath $(BOARD_CONFIG_HEADER)) upload
