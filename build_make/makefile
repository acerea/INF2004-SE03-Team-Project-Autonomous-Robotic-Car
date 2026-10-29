################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# Specify the name of the executable file to be built (Do not include the extension)
EXE_FILE := mtkernel_3

# Specify only one target name
#TARGET := _IOTE_M367_
#TARGET := _IOTE_RX231_
#TARGET := _IOTE_STM32L4_
#TARGET := _IOTE_RZA2M_
#TARGET := _NUCLEO_H723_
TARGET := _PICO_RP2040_

RM := rm -rf

# The subdir.mk files included below define the first explicit rules make
# sees, so a bare 'make' would otherwise pick one of their object files as
# the default goal and quietly leave the image stale.
# wy added this
INCPATH = -I"../include" -I"../config" -I"../kernel/knlinc"

ifeq ($(TEST),1)
CFLAGS += -DAPP_TEST=1
CPPFLAGS += -DAPP_TEST=1
endif

.DEFAULT_GOAL := all

INCPATH = -I"../include" -I"../config" -I"../kernel/knlinc"

-include sources.mk

include mtkernel_3/lib/libtm/subdir.mk
include mtkernel_3/lib/libtk/subdir.mk
include mtkernel_3/kernel/usermain/subdir.mk
include mtkernel_3/kernel/tstdlib/subdir.mk
include mtkernel_3/kernel/tkernel/subdir.mk
include mtkernel_3/kernel/sysinit/subdir.mk
include mtkernel_3/kernel/inittask/subdir.mk

include mtkernel_3/device/subdir.mk

ifeq ($(TARGET), _IOTE_M367_)
include iote_m367.mk
endif
ifeq ($(TARGET), _IOTE_RX231_)
include iote_rx231.mk
endif
ifeq ($(TARGET), _IOTE_STM32L4_)
include iote_stm32l4.mk
endif
ifeq ($(TARGET), _IOTE_RZA2M_)
include iote_rza2m.mk
endif
ifeq ($(TARGET), _NUCLEO_H723_)
include nucleo_h723.mk
endif
ifeq ($(TARGET), _PICO_RP2040_)
include pico_rp2040.mk
endif

-include mtkernel_3/app_program/subdir.mk

# Create every object directory up front.  Git does not store empty
# directories, so a fresh clone lacks them, and not every compile rule
# creates its own (make 3.81, shipped with macOS, picks the first matching
# pattern rule, which may be one without a mkdir).
ifneq ($(MAKECMDGOALS),clean)
$(shell mkdir -p $(sort $(dir $(OBJS))))
endif

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# All EXE_FILE
all: $(EXE_FILE).elf

$(EXE_FILE).elf: $(OBJS)
	@echo 'Linker: $@'
	$(LINK)  $(LFLAGS) -T $(LNKFILE) -Wl,-Map,"$(EXE_FILE).map" -o "$(EXE_FILE).elf" $(OBJS) $(LDLIBS) -lgcc
	@echo 'Finished building target: $@'
ifneq ($(SIZE),)
	$(SIZE) $(EXE_FILE).elf
endif
ifneq ($(E2U),)
	$(E2U) $(EXE_FILE).elf $(EXE_FILE).uf2
endif
	@echo ' '

# Other EXE_FILEs
clean:
	-$(RM) $(OBJS) $(SECONDARY_SIZE) $(ASM_DEPS) $(S_UPPER_DEPS) $(C_DEPS) $(EXE_FILE).elf $(EXE_FILE).map
	-@echo ' '

.PHONY: all clean dependents

# Added this -wy
APP_BINARY = $(EXE_FILE)

flash:
	@if [ ! -d "/Volumes/RPI-RP2" ]; then \
		echo "Plug in Pico with BOOTSEL held down..."; \
		while [ ! -d "/Volumes/RPI-RP2" ]; do sleep 0.5; done; \
	fi
	@echo "Flashing $(APP_BINARY).uf2 via Finder..."
	@osascript -e 'tell application "Finder" to duplicate POSIX file "$(CURDIR)/$(APP_BINARY).uf2" to POSIX file "/Volumes/RPI-RP2" with replacing' > /dev/null 2>&1 || true
	@echo "Flash complete."

run: flash
	@echo "Waiting 3 seconds for Pico serial port to enumerate..."
	@sleep 3
	@PORT=$$(ls /dev/cu.usbmodem* /dev/tty.usbmodem* 2>/dev/null | head -n 1); \
	if [ -n "$$PORT" ]; then \
		echo "Connecting to $$PORT..."; \
		screen $$PORT 115200; \
	else \
		echo "Serial port not found. Check USB connection or open Serial Monitor."; \
	fi