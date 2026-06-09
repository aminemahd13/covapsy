# Makefile for CoVAPSy STM32G431KB firmware.
#
# Build on the Pi:
#   make           -> build/firmware.elf, .bin, .hex
#   make flash     -> build/firmware.bin + st-flash write 0x08000000
#   make clean
#
# Requirements (apt on Ubuntu/Raspbian):
#   gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi stlink-tools

TARGET   = firmware
BUILD    = build

PREFIX   = arm-none-eabi-
CC       = $(PREFIX)gcc
AS       = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY  = $(PREFIX)objcopy
SIZE     = $(PREFIX)size

# --- CPU / FPU --------------------------------------------------------------
CPU      = -mcpu=cortex-m4
FPU      = -mfpu=fpv4-sp-d16
FLOAT    = -mfloat-abi=hard
MCU      = $(CPU) -mthumb $(FPU) $(FLOAT)

# --- Symbols ----------------------------------------------------------------
C_DEFS   = -DUSE_HAL_DRIVER -DSTM32G431xx

# --- Includes ---------------------------------------------------------------
C_INCLUDES = \
  -ICore/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc \
  -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
  -IDrivers/CMSIS/Include

# --- Sources ----------------------------------------------------------------
C_SOURCES = \
  $(wildcard Core/Src/*.c) \
  $(wildcard Drivers/STM32G4xx_HAL_Driver/Src/*.c)

ASM_SOURCES = Core/Startup/startup_stm32g431kbtx.s

# --- Linker -----------------------------------------------------------------
LDSCRIPT = STM32G431KBTX_FLASH.ld
LIBS     = -lc -lm -lnosys
LDFLAGS  = $(MCU) -specs=nano.specs -u _printf_float -T$(LDSCRIPT) $(LIBS) \
           -Wl,-Map=$(BUILD)/$(TARGET).map,--cref -Wl,--gc-sections

# --- C / ASM flags ----------------------------------------------------------
OPT      = -O2
CFLAGS   = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2
ASFLAGS  = $(MCU) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2

# --- Object list ------------------------------------------------------------
OBJECTS  = $(addprefix $(BUILD)/, $(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD)/, $(notdir $(ASM_SOURCES:.s=.o)))

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# --- Targets ----------------------------------------------------------------
.PHONY: all flash clean

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD):
	mkdir -p $@

flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x08000000

clean:
	rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)
