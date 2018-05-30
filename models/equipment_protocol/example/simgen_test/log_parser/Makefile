BIN_IMAGE = simgen_udp_log_parser
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
EQUIPMENT_PROTOCOL_DIR := $(patsubst %/example/simgen_test/log_parser/Makefile, %, $(MKFILE_PATH))
$(info MKFILE_PATH = $(MKFILE_PATH))
###### C flags #####
CC = gcc
CFLAGS = -Wall -g -MD -save-temps
CFLAGS += -I./ \
	-I$(EQUIPMENT_PROTOCOL_DIR)/include

##### C Source #####

C_SOURCES += $(EQUIPMENT_PROTOCOL_DIR)/example/simgen_test/log_parser/simgen_udp_log_parser.c
##### OBJECTS #####
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))
all: $(BIN_IMAGE) $(C_SOURCES)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BIN_IMAGE): $(OBJECTS)
	$(CC) -Wall -g $(C_SOURCES) -o $@ $(CFLAGS)

-include $(OBJECTS:.o=.d)

.PHONY : clean
clean:
	rm -f $(BIN_IMAGE)
	find ../../ -name "*.o" -type f -delete
	find ../../ -name "*.d" -type f -delete
	find ../../ -name "*.i" -type f -delete
	find ../../ -name "*.s" -type f -delete