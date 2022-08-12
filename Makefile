RM := rm -rf

COMPILER_PREFIX ?= riscv-none-embed

# Define option(s) defined in pre-processor compiler option(s)
DEFINE_OPTS = -DDEBUG=1
# Optimisation option(s)
OPTIM_OPTS = -O3
# Debug option(s)
DEBUG = -g

BUILD_DIR = ./build

PROJECT = $(BUILD_DIR)/ch569-usb3-hspi-bridge

RVMSIS_DIR  = ./wch-ch56x-bsp/rvmsis
RVMSIS_SRCS = $(wildcard $(RVMSIS_DIR)/*.c)
OBJS       += $(patsubst $(RVMSIS_DIR)/%.c,$(BUILD_DIR)/%.o,$(RVMSIS_SRCS))

DRV_DIR   = ./wch-ch56x-bsp/drv
DRV_SRCS  = $(wildcard $(DRV_DIR)/*.c)
OBJS     += $(patsubst $(DRV_DIR)/%.c,$(BUILD_DIR)/%.o,$(DRV_SRCS))

USER_DIR  = ./User
USER_SRCS = $(wildcard $(USER_DIR)/*.c)
OBJS     += $(patsubst $(USER_DIR)/%.c,$(BUILD_DIR)/%.o,$(USER_SRCS))

USB20_DIR  = ./USB20
USB20_SRCS = $(wildcard $(USB20_DIR)/*.c)
OBJS     += $(patsubst $(USB20_DIR)/%.c,$(BUILD_DIR)/%.o,$(USB20_SRCS))

USB30_DIR  = ./USB30
USB30_SRCS = $(wildcard $(USB30_DIR)/*.c)
OBJS     += $(patsubst $(USB30_DIR)/%.c,$(BUILD_DIR)/%.o,$(USB30_SRCS))

# All of the sources participating in the build are defined here
OBJS += $(BUILD_DIR)/startup_CH56x.o
DEPS  = $(subst .o,.d,$(OBJS))
LIBS  = $(USB30_DIR)/libCH56x_usb30.a

BASE_OPTS = -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 $(OPTIM_OPTS) -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections
C_OPTS    = $(BASE_OPTS) $(DEBUG) $(DEFINE_OPTS)\
              $(INCLUDES) -std=gnu99 -MMD -MP -MT"$(@)"
LD_OPTS   = -T ".ld" -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"$(PROJECT).map" --specs=nano.specs --specs=nosys.specs

INCLUDES = \
  -I"$(RVMSIS_DIR)" \
  -I"$(DRV_DIR)" \
  -I"$(USER_DIR)" \
  -I"$(USB20_DIR)" \
  -I"$(USB30_DIR)" \

# Add inputs and outputs from these tool invocations to the build variables
SECONDARY_FLASH += $(PROJECT).hex $(PROJECT).bin
SECONDARY_LIST  += $(PROJECT).lst
SECONDARY_SIZE  += $(PROJECT).siz
SECONDARY_MAP   += $(PROJECT).map

SECONDARY_OUTPUTS = $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE) $(SECONDARY_MAP)
secondary-outputs: $(SECONDARY_OUTPUTS)

# All Target
all: $(PROJECT).elf secondary-outputs

.PRECIOUS: $(BUILD_DIR)/. $(BUILD_DIR)%/.

$(BUILD_DIR)/.:
	mkdir -p $@

$(BUILD_DIR)%/.:
	mkdir -p $@

.SECONDEXPANSION:

$(BUILD_DIR)/startup_CH56x.o: ./wch-ch56x-bsp/startup/startup_CH56x.S | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -x assembler -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(USER_DIR)/%.c | $$(@D)/.
	@echo $(OBJS)
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(USB20_DIR)/%.c | $$(@D)/.
	@echo $(OBJS)
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(USB30_DIR)/%.c | $$(@D)/.
	@echo $(OBJS)
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(RVMSIS_DIR)/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: $(DRV_DIR)/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

# Tool invocations
$(PROJECT).elf: $(OBJS)
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	$(COMPILER_PREFIX)-gcc $(BASE_OPTS) $(LD_OPTS) -o "$(PROJECT).elf" $(OBJS) $(LIBS)
	@echo ' '

$(PROJECT).hex: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	$(COMPILER_PREFIX)-objcopy -O ihex "$(PROJECT).elf"  "$(PROJECT).hex"
	@echo ' '

$(PROJECT).bin: $(PROJECT).elf
	-@echo 'Create Flash Image BIN'
	-$(COMPILER_PREFIX)-objcopy -O binary "$(PROJECT).elf"  "$(PROJECT).bin"
	-@echo ' '

$(PROJECT).lst: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	$(COMPILER_PREFIX)-objdump --source --all-headers --demangle --line-numbers --wide "$(PROJECT).elf" > "$(PROJECT).lst"
	@echo ' '

$(PROJECT).siz: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	$(COMPILER_PREFIX)-size --format=berkeley "$(PROJECT).elf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) $(DEPS) $(SECONDARY_OUTPUTS) $(PROJECT).elf
	-@echo ' '

.PHONY: all clean dependents
