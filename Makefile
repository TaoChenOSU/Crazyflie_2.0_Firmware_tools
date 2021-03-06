# Makefile to compile tools to run some of the crazyflie modules on
# my PC.

# Compiler options
CC=gcc
CFLAGS=-c -Wall -std=c99
CFLAG_LM=-lm

# Folders
BIN		= bin
CF_SOURCE	= crazyflie-firmware/src

# add crazyflie firmware modules header files
INCLUDES += -I$(CF_SOURCE)/modules/interface
INCLUDES += -I$(CF_SOURCE)/hal/interface
INCLUDES += -Isrc/include

gater_data: controller_mellinger.o gather_data_from_mellinger_ctrl.o random_number_generator.o
	$(CC) -o gather_data controller_mellinger.o gather_data_from_mellinger_ctrl.o random_number_generator.o $(CFLAG_LM)

controller_mellinger.o: $(CF_SOURCE)/modules/src/controller_mellinger.c
	$(CC) $(CFLAGS) $(INCLUDES) $(CF_SOURCE)/modules/src/controller_mellinger.c

gather_data_from_mellinger_ctrl.o: src/gather_data_from_mellinger_ctrl.c
	$(CC) $(CFLAGS) $(INCLUDES) src/gather_data_from_mellinger_ctrl.c

random_number_generator.o: src/random_number_generator.c
	$(CC) $(CFLAGS) $(INCLUDES) src/random_number_generator.c

clean:
	rm controller_mellinger.o gather_data_from_mellinger_ctrl.o random_number_generator.o
