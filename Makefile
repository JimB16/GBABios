#---------------------------------------------------------------------------------
# Clear the implicit built in rules
#---------------------------------------------------------------------------------
.SUFFIXES:
#---------------------------------------------------------------------------------
ifeq ($(strip $(DEVKITARM)),)
$(error "Please set DEVKITARM in your environment. export DEVKITARM=<path to>devkitARM)
endif


export TARGET		:=	$(shell basename $(CURDIR))

include $(DEVKITARM)/base_rules


all: $(TARGET).gba

#---------------------------------------------------------------------------------
%.gba: %.elf
	$(DEVKITARM)/bin/arm-none-eabi-objcopy -v -O binary $< $@
	@echo built ... $(notdir $@)

#---------------------------------------------------------------------------------
%.elf: GBABios.o
	@echo linking cartridge
	$(DEVKITARM)/bin/arm-none-eabi-ld  -T lnkscript -Map GBABios.map GBABios.o -o $@

#---------------------------------------------------------------------------------
%.o:
	$(DEVKITARM)/bin/arm-none-eabi-as -mcpu=arm7tdmi -X -mthumb-interwork GBABios.s -o $@

