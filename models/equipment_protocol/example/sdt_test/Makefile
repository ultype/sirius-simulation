BIN_IMAGE = sdt_test
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
EQUIPMENT_PROTOCOL_DIR := $(patsubst %/example/sdt_test/Makefile, %, $(MKFILE_PATH))
SIRIUS_HOME = $(patsubst %/models/equipment_protocol, %, $(EQUIPMENT_PROTOCOL_DIR))
$(info MKFILE_PATH = $(MKFILE_PATH))
$(info SIRIUS_HOME = $(SIRIUS_HOME))
###### C flags #####
CC = gcc
CFLAGS = -Wall -g
CFLAGS += -I./ \
		  -I$(EQUIPMENT_PROTOCOL_DIR)/include \
		  -I$(SIRIUS_HOME)/models/icf/include

##### C Source #####
MODEL_C_SOURCE = $(SIRIUS_HOME)/models/icf/src/rs422_serialport.c\
				 $(SIRIUS_HOME)/models/icf/src/icf_utility.c

C_SOURCES += $(EQUIPMENT_PROTOCOL_DIR)/example/sdt_test/sdt_test.c
##### OBJECTS #####
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))
OBJECTS += $(patsubst %.c, %.o, $(MODEL_C_SOURCE))
all: $(MODEL_C_SOURCE) $(BIN_IMAGE) $(C_SOURCES)

deps := $(OBJECTS:%.o=%.o.d)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BIN_IMAGE): $(OBJECTS)
	$(CC) -Wall -g $(C_SOURCES) $(MODEL_C_SOURCE) -o $@ $(CFLAGS)
.PHONY : clean
clean:
	rm -f $(BIN_IMAGE)
	find ../../ -name "*.o" -type f -delete
	find ../../ -name "*.d" -type f -delete