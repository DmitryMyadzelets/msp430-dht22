# http://mrbook.org/blog/tutorials/make/

OBJECTS = main.o dht22.o

GCC_DIR = ~/ti/usr/local/bin
SUPPORT_FILE_DIRECTORY = ~/ti/msp430-gcc-support-files/include

DEVICE  = msp430g2553
CC      = $(GCC_DIR)/msp430-elf-gcc
GDB     = $(GCC_DIR)/msp430-elf-gdb

CFLAGS = -I $(SUPPORT_FILE_DIRECTORY) -mmcu=$(DEVICE) -O2 -g
LFLAGS = -L $(SUPPORT_FILE_DIRECTORY)

all: ${OBJECTS}
	$(CC) $(CFLAGS) $(LFLAGS) $? -o $(DEVICE).out

debug: all
	$(GDB) $(DEVICE).out

clear: 
	rm ${OBJECTS} $(DEVICE).out

install:
	mspdebug rf2500

