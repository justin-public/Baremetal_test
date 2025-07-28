CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0
LDFLAGS= -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nosys.specs -T stm32_ls.ld -Wl,-Map=test.map

all:main.o stm32_startup.o test.elf test.hex

main.o:main.c
	$(CC) $(CFLAGS) -o $@ $^

stm32_startup.o:stm32_startup.c
	$(CC) $(CFLAGS) -o $@ $^

test.hex: test.elf
	$(OBJCOPY) -O ihex $< $@

test.elf: main.o stm32_startup.o
	$(CC) $(LDFLAGS) -o $@ $^

clean:
		rm -rf *.o *.elf *.hex *.map

load:
	openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg