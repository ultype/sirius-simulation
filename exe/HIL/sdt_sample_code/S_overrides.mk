
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CANNON_HOME = $(patsubst %/exe/HIL/sdt_sample_code/S_overrides.mk, %, $(MKFILE_PATH))

$(info MKFILE_PATH = $(MKFILE_PATH))
$(info CANNON_HOME = $(CANNON_HOME))

INCLUDES = -I${CANNON_HOME}/models/clock_source_mgmt

TRICK_CFLAGS += ${INCLUDES} -g -D_GNU_SOURCE
TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
TRICK_CXXFLAGS += --std=c++11 ${INCLUDES} -g
TRICK_CXXFLAGS += -Wall -Wextra -Wshadow -Wno-narrowing
TRICK_USER_LINK_LIBS += -larmadillo -lboost_serialization -lbiodaq
MAKEFLAGS += -j16
