BIN_IMAGE = simgen_test
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
EQUIPMENT_PROTOCOL_DIR := $(patsubst %/example/simgen_test/Makefile, %, $(MKFILE_PATH))
$(info MKFILE_PATH = $(MKFILE_PATH))
###### C flags #####
CC = gcc
CFLAGS = -Wall -g
CFLAGS += -I./ \
	-I$(EQUIPMENT_PROTOCOL_DIR)/include

##### C Source #####
MODEL_C_SOURCE = $(EQUIPMENT_PROTOCOL_DIR)/src/simgen_remote.c

C_SOURCES += $(EQUIPMENT_PROTOCOL_DIR)/example/simgen_test/simgen_test.c
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