BIN_IMAGE = server
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
ICF_MODEL_DIR := $(patsubst %/QA_Tool/EGSE_System_Profiling_Server/server-client/Makefile, %, $(MKFILE_PATH))
BUILD_DIR := $(ICF_MODEL_DIR)/QA_Tool/EGSE_System_Profiling_Server/server-client/build
$(info MKFILE_PATH = $(MKFILE_PATH))
$(info BUILD_DIR = $(BUILD_DIR))
###### C flags #####
CC = gcc
CFLAGS = -Wall -g
CFLAGS += -I./ \
	-I$(BUILD_DIR)/include/
CFLAGS +=-pthread

##### C Source #####
MODEL_C_SOURCE = $(ICF_MODEL_DIR)/src/ethernet.c \
		$(ICF_MODEL_DIR)/src/icf_utility.c

C_SOURCES = $(subst src,QA_Tool/EGSE_System_Profiling_Server/server-client/build/src,$(MODEL_C_SOURCE))
C_SOURCES += $(ICF_MODEL_DIR)/QA_Tool/EGSE_System_Profiling_Server/server-client/server.c
##### OBJECTS #####
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

all: $(MODEL_C_SOURCE) $(BUILD_DIR) $(BIN_IMAGE) $(C_SOURCES)

$(BUILD_DIR): $(MODEL_C_SOURCE)
	rm -rf $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)
	sh ../build_script.sh  $(ICF_MODEL_DIR)/src $(BUILD_DIR)
	sh ../build_script.sh  $(ICF_MODEL_DIR)/include $(BUILD_DIR)

deps := $(OBJECTS:%.o=%.o.d)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BIN_IMAGE): $(OBJECTS)
	$(CC) -Wall -g $(C_SOURCES) -o $@ -lrt $(CFLAGS)
	rm -rf $(BUILD_DIR)
.PHONY : clean
clean:
	rm -f $(BIN_IMAGE)
	rm -rf $(BUILD_DIR)
	find . -name "*.o" -type f -delete
	find . -name "*.d" -type f -delete
