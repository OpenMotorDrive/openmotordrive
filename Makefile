ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS := --specs=nosys.specs --static -nostartfiles -Llibopencm3/lib -Tstm32f3.ld -Wl,-Map=bootloader.map -Wl,--gc-sections
LDLIBS := -lopencm3_stm32f3 -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

bootloader.elf bootloader.map: bootloader.o
	+make -C libopencm3 lib
	arm-none-eabi-g++ -o bootloader.elf $(LDFLAGS) $(ARCH_FLAGS) bootloader.o $(LDLIBS)

%.o: %.cpp
	arm-none-eabi-g++ -c -Ilibopencm3/include $(ARCH_FLAGS) -DSTM32F3 $<

clean:
	make -C libopencm3 clean
	rm -f *.o *.elf *.map
