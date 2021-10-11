# tools

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# this is ST's free STM32CubeProgrammer package. if you installed it under sudo then it should be in the location below

PROGRAMMER = /usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer.sh
             
# flags for tools

INCLUDE = -ICore/Inc -IDrivers/STM32WBxx_HAL_Driver/Inc -IDrivers/STM32WBxx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32WBxx/Include -IDrivers/CMSIS/Include -IMiddlewares/ST/STM32_WPAN/ble/core/template -IMiddlewares/ST/STM32_WPAN/ble/core -IUtilities/lpm/tiny_lpm -ISTM32_WPAN/App -IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -IMiddlewares/ST/STM32_WPAN/ble/svc/Inc -IMiddlewares/ST/STM32_WPAN/ble -IMiddlewares/ST/STM32_WPAN/ble/core/auto -IUtilities/sequencer -IMiddlewares/ST/STM32_WPAN/utilities -IMiddlewares/ST/STM32_WPAN/ble/svc/Src -IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -IMiddlewares/ST/STM32_WPAN -IMiddlewares/ST/STM32_WPAN/interface/patterns/ble_thread
CFLAGS = -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32WB55xx  -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
LDFLAGS = -mcpu=cortex-m4 --specs=nosys.specs -Wl,-Map=build/bluetooth-temperature.map -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
ASMFLAGS = -mcpu=cortex-m4 -g3 -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb

# your targets:
#   'make release' for the optimised build (same as just 'make')
#   'make debug' for a build with symbols and no optimisation
# include the 'flash' target to write to your device connected with ST-Link, e.g:
#   'make release flash'
#   'make debug flash'
# if switching between 'release' and 'debug' then do a 'make clean' first, or delete the 'build' directory.

release: CFLAGS += -O3
debug: CFLAGS += -DDEBUG -g3 -O0

release: hex bin lst size
debug: hex bin lst size

# C, C++ and assembly sources

CSRC := $(shell find . -name "*.c")
CPPSRC := $(shell find . -name "*.cpp")
ASMSRC := $(shell find . -name "*.s")

# equivalent objects for the sources

OBJ := $(CSRC:%.c=build/%.o) $(CPPSRC:%.cpp=build/%.o) $(ASMSRC:%.s=build/%.o)

# everything gets built into a 'build' sub-directory

build/%.o: %.c
	mkdir -p "$(@D)"
	$(CC) $(CFLAGS) ${INCLUDE} -c $< -o $@

build/%.o: %.cpp
	mkdir -p "$(@D)"
	$(CC) $(CFLAGS) ${INCLUDE} -c $< -o $@

build/%.o: %.s
	mkdir -p "$(@D)"
	$(CC) $(ASMFLAGS) ${INCLUDE} -c $< -o $@

# linker

elf: $(OBJ)
	$(CC) -o build/bluetooth-temperature.elf $(OBJ) -TSTM32WB55CCUX_FLASH.ld $(LDFLAGS)

# convert elf to ihex for flashing

hex: elf
	$(OBJCOPY) -O ihex build/bluetooth-temperature.elf build/bluetooth-temperature.hex

# binary

bin: elf
	$(OBJCOPY) -O binary build/bluetooth-temperature.elf build/bluetooth-temperature.bin

# assembly listing

lst: elf
	$(OBJDUMP) -h -S build/bluetooth-temperature.elf > build/bluetooth-temperature.lst

# size information to show MCU memory utilisation

size: elf
	$(SIZE) --format=berkeley build/bluetooth-temperature.elf | tee build/bluetooth-temperature.size

# programmer (flags = use SWD, connect under reset, hardware reset, program, verify, reset-and-run)

flash: elf
	$(PROGRAMMER) -c port=SWD mode=UR reset=HWrst -d build/bluetooth-temperature.elf -v -hardRst

# clean up

clean:
	rm -rf build
